import torch
from torch import nn

from einops import rearrange


class PreNormResidual(nn.Module):
    def __init__(self, dim, fn):
        super().__init__()
        self.norm = nn.LayerNorm(dim)
        self.fn = fn

    def forward(self, x, **kwargs):
        return self.fn(self.norm(x), **kwargs) + x


class PreNorm(nn.Module):
    def __init__(self, dim, fn):
        super().__init__()
        self.norm = nn.LayerNorm(dim)
        self.fn = fn

    def forward(self, x, **kwargs):
        return self.fn(self.norm(x), **kwargs)


class FeedForward(nn.Module):
    def __init__(self, dim, hidden_dim, dropout=0.):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(dim, hidden_dim),
            nn.GELU(),
            nn.Dropout(dropout),
            nn.Linear(hidden_dim, dim),
            nn.Dropout(dropout)
        )

    def forward(self, x):
        return self.net(x)


class CavAttention(nn.Module):
    """
    Vanilla CAV attention.
    """
    def __init__(self, dim, heads, dim_head=64, dropout=0.1):
        super().__init__()
        inner_dim = heads * dim_head

        self.heads = heads
        self.scale = dim_head ** -0.5

        self.attend = nn.Softmax(dim=-1)
        self.to_qkv = nn.Linear(dim, inner_dim * 3, bias=False)

        self.to_out = nn.Sequential(
            nn.Linear(inner_dim, dim),
            nn.Dropout(dropout)
        )

    def forward(self, x, mask, prior_encoding):
        # x: (B, L, H, W, C) -> (B, H, W, L, C)
        # mask: (B, L)
        x = x.permute(0, 2, 3, 1, 4)
        # mask: (B, 1, H, W, L, 1)
        mask = mask.unsqueeze(1)

        # qkv: [(B, H, W, L, C_inner) *3]
        qkv = self.to_qkv(x).chunk(3, dim=-1)
        # q: (B, M, H, W, L, C)
        q, k, v = map(lambda t: rearrange(t, 'b h w l (m c) -> b m h w l c',
                                          m=self.heads), qkv)

        # attention, (B, M, H, W, L, L)
        att_map = torch.einsum('b m h w i c, b m h w j c -> b m h w i j',
                               q, k) * self.scale
        # add mask
        att_map = att_map.masked_fill(mask == 0, -float('inf'))
        # softmax
        att_map = self.attend(att_map)

        # out:(B, M, H, W, L, C_head)
        out = torch.einsum('b m h w i j, b m h w j c -> b m h w i c', att_map,
                           v)
        out = rearrange(out, 'b m h w l c -> b h w l (m c)',
                        m=self.heads)
        out = self.to_out(out)
        # (B L H W C)
        out = out.permute(0, 3, 1, 2, 4)
        return out


class BaseEncoder(nn.Module):
    def __init__(self, dim, depth, heads, dim_head, mlp_dim, dropout=0.):
        super().__init__()
        self.layers = nn.ModuleList([])
        for _ in range(depth):
            self.layers.append(nn.ModuleList([
                PreNorm(dim, CavAttention(dim,
                                          heads=heads,
                                          dim_head=dim_head,
                                          dropout=dropout)),
                PreNorm(dim, FeedForward(dim, mlp_dim, dropout=dropout))
            ]))

    def forward(self, x, mask):
        for attn, ff in self.layers:
            x = attn(x, mask=mask) + x
            x = ff(x) + x
        return x


class BaseTransformer(nn.Module):
    def __init__(self, args):
        super().__init__()

        dim = args['dim']
        depth = args['depth']
        heads = args['heads']
        dim_head = args['dim_head']
        mlp_dim = args['mlp_dim']
        dropout = args['dropout']
        max_cav = args['max_cav']

        self.encoder = BaseEncoder(dim, depth, heads, dim_head, mlp_dim,
                                   dropout)

    def forward(self, x, mask):
        # B, L, H, W, C
        output = self.encoder(x, mask)
        # B, H, W, C
        output = output[:, 0]

        return output