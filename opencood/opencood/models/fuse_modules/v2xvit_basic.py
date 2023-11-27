import math

from opencood.models.sub_modules.base_transformer import *
from opencood.models.fuse_modules.hmsa import *
from opencood.models.fuse_modules.mswin import *
from opencood.models.sub_modules.torch_transformation_utils import \
    get_transformation_matrix, warp_affine, get_roi_and_cav_mask, \
    get_discretized_transformation_matrix


class STTF(nn.Module):
    def __init__(self, args):
        super(STTF, self).__init__()
        self.discrete_ratio = args['voxel_size'][0]
        self.downsample_rate = args['downsample_rate']

    def forward(self, x, mask, spatial_correction_matrix):
        x = x.permute(0, 1, 4, 2, 3)
        dist_correction_matrix = get_discretized_transformation_matrix(
            spatial_correction_matrix, self.discrete_ratio,
            self.downsample_rate)
        # Only compensate non-ego vehicles
        B, L, C, H, W = x.shape

        T = get_transformation_matrix(
            dist_correction_matrix[:, 1:, :, :].reshape(-1, 2, 3), (H, W))
        cav_features = warp_affine(x[:, 1:, :, :, :].reshape(-1, C, H, W), T,
                                   (H, W))
        cav_features = cav_features.reshape(B, -1, C, H, W)
        x = torch.cat([x[:, 0, :, :, :].unsqueeze(1), cav_features], dim=1)
        x = x.permute(0, 1, 3, 4, 2)
        return x


class RelTemporalEncoding(nn.Module):
    """
    Implement the Temporal Encoding (Sinusoid) function.
    """

    def __init__(self, n_hid, RTE_ratio, max_len=100, dropout=0.2):
        super(RelTemporalEncoding, self).__init__()
        position = torch.arange(0., max_len).unsqueeze(1)
        div_term = torch.exp(torch.arange(0, n_hid, 2) *
                             -(math.log(10000.0) / n_hid))
        emb = nn.Embedding(max_len, n_hid)
        emb.weight.data[:, 0::2] = torch.sin(position * div_term) / math.sqrt(
            n_hid)
        emb.weight.data[:, 1::2] = torch.cos(position * div_term) / math.sqrt(
            n_hid)
        emb.requires_grad = False
        self.RTE_ratio = RTE_ratio
        self.emb = emb
        self.lin = nn.Linear(n_hid, n_hid)

    def forward(self, x, t):
        # When t has unit of 50ms, rte_ratio=1.
        # So we can train on 100ms but test on 50ms
        return x + self.lin(self.emb(t * self.RTE_ratio)).unsqueeze(
            0).unsqueeze(1)


class RTE(nn.Module):
    def __init__(self, dim, RTE_ratio=2):
        super(RTE, self).__init__()
        self.RTE_ratio = RTE_ratio

        self.emb = RelTemporalEncoding(dim, RTE_ratio=self.RTE_ratio)

    def forward(self, x, dts):
        # x: (B,L,H,W,C)
        # dts: (B,L)
        rte_batch = []
        for b in range(x.shape[0]):
            rte_list = []
            for i in range(x.shape[1]):
                rte_list.append(
                    self.emb(x[b, i, :, :, :], dts[b, i]).unsqueeze(0))
            rte_batch.append(torch.cat(rte_list, dim=0).unsqueeze(0))
        return torch.cat(rte_batch, dim=0)


class V2XFusionBlock(nn.Module):
    def __init__(self, num_blocks, cav_att_config, pwindow_config):
        super().__init__()
        # first multi-agent attention and then multi-window attention
        self.layers = nn.ModuleList([])
        self.num_blocks = num_blocks

        for _ in range(num_blocks):
            att = HGTCavAttention(cav_att_config['dim'],
                                  heads=cav_att_config['heads'],
                                  dim_head=cav_att_config['dim_head'],
                                  dropout=cav_att_config['dropout']) if \
                cav_att_config['use_hetero'] else \
                CavAttention(cav_att_config['dim'],
                             heads=cav_att_config['heads'],
                             dim_head=cav_att_config['dim_head'],
                             dropout=cav_att_config['dropout'])
            self.layers.append(nn.ModuleList([
                PreNorm(cav_att_config['dim'], att),
                PreNorm(cav_att_config['dim'],
                        PyramidWindowAttention(pwindow_config['dim'],
                                               heads=pwindow_config['heads'],
                                               dim_heads=pwindow_config[
                                                   'dim_head'],
                                               drop_out=pwindow_config[
                                                   'dropout'],
                                               window_size=pwindow_config[
                                                   'window_size'],
                                               relative_pos_embedding=
                                               pwindow_config[
                                                   'relative_pos_embedding'],
                                               fuse_method=pwindow_config[
                                                   'fusion_method']))]))

    def forward(self, x, mask, prior_encoding):
        for cav_attn, pwindow_attn in self.layers:
            x = cav_attn(x, mask=mask, prior_encoding=prior_encoding) + x
            x = pwindow_attn(x) + x
        return x


class V2XTEncoder(nn.Module):
    def __init__(self, args):
        super().__init__()

        cav_att_config = args['cav_att_config']
        pwindow_att_config = args['pwindow_att_config']
        feed_config = args['feed_forward']

        num_blocks = args['num_blocks']
        depth = args['depth']
        mlp_dim = feed_config['mlp_dim']
        dropout = feed_config['dropout']

        self.downsample_rate = args['sttf']['downsample_rate']
        self.discrete_ratio = args['sttf']['voxel_size'][0]
        self.use_roi_mask = args['use_roi_mask']
        self.use_RTE = cav_att_config['use_RTE']
        self.RTE_ratio = cav_att_config['RTE_ratio']
        self.sttf = STTF(args['sttf'])
        # adjust the channel numbers from 256+3 -> 256
        self.prior_feed = nn.Linear(cav_att_config['dim'] + 3,
                                    cav_att_config['dim'])
        self.layers = nn.ModuleList([])
        if self.use_RTE:
            self.rte = RTE(cav_att_config['dim'], self.RTE_ratio)
        for _ in range(depth):
            self.layers.append(nn.ModuleList([
                V2XFusionBlock(num_blocks, cav_att_config, pwindow_att_config),
                PreNorm(cav_att_config['dim'],
                        FeedForward(cav_att_config['dim'], mlp_dim,
                                    dropout=dropout))
            ]))

    def forward(self, x, mask, spatial_correction_matrix):

        # transform the features to the current timestamp
        # velocity, time_delay, infra
        # (B,L,H,W,3)
        prior_encoding = x[..., -3:]
        # (B,L,H,W,C)
        x = x[..., :-3]
        if self.use_RTE:
            # dt: (B,L)
            dt = prior_encoding[:, :, 0, 0, 1].to(torch.int)
            x = self.rte(x, dt)
        x = self.sttf(x, mask, spatial_correction_matrix)
        com_mask = mask.unsqueeze(1).unsqueeze(2).unsqueeze(
            3) if not self.use_roi_mask else get_roi_and_cav_mask(x.shape,
                                                                  mask,
                                                                  spatial_correction_matrix,
                                                                  self.discrete_ratio,
                                                                  self.downsample_rate)
        for attn, ff in self.layers:
            x = attn(x, mask=com_mask, prior_encoding=prior_encoding)
            x = ff(x) + x
        return x


class V2XTransformer(nn.Module):
    def __init__(self, args):
        super(V2XTransformer, self).__init__()

        encoder_args = args['encoder']
        self.encoder = V2XTEncoder(encoder_args)

    def forward(self, x, mask, spatial_correction_matrix):
        output = self.encoder(x, mask, spatial_correction_matrix)
        output = output[:, 0]
        return output
