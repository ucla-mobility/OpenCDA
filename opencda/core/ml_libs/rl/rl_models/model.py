import torch
import torch.nn as nn
from typing import List, Tuple, Union, Dict, Optional

from opencda.core.ml_libs.rl.utils.img_encoder import BEVSpeedConvEncoder
from ding.model.common.head import DuelingHead, RegressionHead, ReparameterizationHead, MultiHead, DiscreteHead


class DQNRLModel(nn.Module):

    def __init__(
            self,
            obs_shape: Tuple = [5, 32, 32],
            action_shape: Union[int, Tuple] = 9,
            encoder_hidden_size_list: Tuple = [64, 128, 256],
            dueling: bool = True,
            head_hidden_size: Optional[int] = 512,
            head_layer_num: int = 1,
            activation: Optional[nn.Module] = nn.ReLU(),
            norm_type: Optional[str] = None
    ) -> None:
        super().__init__()
        self._encoder = BEVSpeedConvEncoder(obs_shape, encoder_hidden_size_list, head_hidden_size, [3, 3, 3], [2, 2, 2])
        if dueling:
            head_cls = DuelingHead
        else:
            head_cls = DiscreteHead
        multi_head = not isinstance(action_shape, int)
        if multi_head:
            self._head = MultiHead(
                head_cls,
                head_hidden_size,
                action_shape,
                layer_num=head_layer_num,
                activation=activation,
                norm_type=norm_type
            )
        else:
            self._head = head_cls(
                head_hidden_size, action_shape, head_layer_num, activation=activation, norm_type=norm_type
            )

    def forward(self, obs):
        x = self._encoder(obs)
        y = self._head(x)
        return y


class DDPGRLModel(nn.Module):

    def __init__(
            self,
            obs_shape: Tuple = [5, 32, 32],
            action_shape: Union[int, tuple] = 2,
            share_encoder: bool = False,
            encoder_hidden_size_list: List = [64, 128, 256],
            encoder_embedding_size: int = 512,
            twin_critic: bool = False,
            actor_head_hidden_size: int = 512,
            actor_head_layer_num: int = 1,
            critic_head_hidden_size: int = 512,
            critic_head_layer_num: int = 1,
            activation: Optional[nn.Module] = nn.ReLU(),
            norm_type: Optional[str] = None,
    ) -> None:
        super().__init__()
        self._obs_shape = obs_shape
        self._act_shape = action_shape
        self.twin_critic = twin_critic
        self.share_encoder = share_encoder
        if self.share_encoder:
            self.actor_encoder = self.critic_encoder = BEVSpeedConvEncoder(
                self._obs_shape, encoder_hidden_size_list, encoder_embedding_size, [3, 3, 3], [2, 2, 2]
            )
        else:
            self.actor_encoder = BEVSpeedConvEncoder(
                self._obs_shape, encoder_hidden_size_list, encoder_embedding_size, [3, 3, 3], [2, 2, 2]
            )
            self.critic_encoder = BEVSpeedConvEncoder(
                self._obs_shape, encoder_hidden_size_list, encoder_embedding_size, [3, 3, 3], [2, 2, 2]
            )

        self.actor_head = nn.Sequential(
            nn.Linear(encoder_embedding_size, actor_head_hidden_size), activation,
            RegressionHead(
                actor_head_hidden_size,
                action_shape,
                actor_head_layer_num,
                final_tanh=True,
                activation=activation,
                norm_type=norm_type
            )
        )
        self.twin_critic = twin_critic
        if self.twin_critic:
            if not self.share_encoder:
                self._twin_encoder = BEVSpeedConvEncoder(self._obs_shape, encoder_hidden_size_list, encoder_embedding_size, [3, 3, 3], [2, 2, 2])
            else:
                self._twin_encoder = self.actor_encoder
            self.critic_head = [
                nn.Sequential(
                    nn.Linear(encoder_embedding_size + self._act_shape, critic_head_hidden_size), activation,
                    RegressionHead(
                        critic_head_hidden_size,
                        1,
                        critic_head_layer_num,
                        final_tanh=False,
                        activation=activation,
                        norm_type=norm_type
                    )
                ) for _ in range(2)
            ]
        else:
            self.critic_head = nn.Sequential(
                nn.Linear(encoder_embedding_size + self._act_shape, critic_head_hidden_size), activation,
                RegressionHead(
                    critic_head_hidden_size,
                    1,
                    critic_head_layer_num,
                    final_tanh=False,
                    activation=activation,
                    norm_type=norm_type
                )
            )
        # for convenience of call some apis(such as: self.critic.parameters()), but may cause
        # misunderstanding when print(self)
        self.actor = nn.ModuleList([self.actor_encoder, self.actor_head])
        if self.twin_critic:
            self.critic = nn.ModuleList([self.critic_encoder, *self.critic_head, self._twin_encoder])
        else:
            self.critic = nn.ModuleList([self.critic_encoder, self.critic_head])

    def forward(self, inputs, mode=None, **kwargs):
        assert (mode in ['compute_actor_critic', 'compute_actor', 'compute_critic'])
        f = getattr(self, mode)
        return f(inputs, **kwargs)

    def compute_critic(self, inputs: Dict) -> Dict:
        x0 = self.critic_encoder(inputs['obs'])
        x0 = torch.cat([x0, inputs['action']], dim=1)
        if self.twin_critic:
            x1 = self._twin_encoder(inputs['obs'])
            x1 = torch.cat([x1, inputs['action']], dim=1)
            x = [m(xi)['pred'] for m, xi in [(self.critic_head[0], x0), (self.critic_head[1], x1)]]
        else:
            x = self.critic_head(x0)['pred']
        return {'q_value': x}

    def compute_actor(self, inputs: Dict) -> Dict:
        x = self.actor_encoder(inputs)
        action = self.actor_head(x)['pred']
        return {'action': action}


class TD3RLModel(DDPGRLModel):

    def __init__(
            self,
            obs_shape: Tuple = [5, 32, 32],
            action_shape: Union[int, tuple] = 2,
            share_encoder: bool = False,
            encoder_hidden_size_list: List = [64, 128, 256],
            encoder_embedding_size: int = 512,
            twin_critic: bool = True,
            actor_head_hidden_size: int = 512,
            actor_head_layer_num: int = 1,
            critic_head_hidden_size: int = 512,
            critic_head_layer_num: int = 1,
            activation: Optional[nn.Module] = nn.ReLU(),
            norm_type: Optional[str] = None,
    ) -> None:
        super().__init__(
            obs_shape, action_shape, share_encoder, encoder_hidden_size_list, encoder_embedding_size,
            twin_critic, actor_head_hidden_size, actor_head_layer_num, critic_head_hidden_size,
            critic_head_layer_num, activation, norm_type)
        assert twin_critic


class SACRLModel(nn.Module):

    def __init__(
            self,
            obs_shape: Tuple = [5, 32, 32],
            action_shape: Union[int, tuple] = 2,
            share_encoder: bool = False,
            encoder_hidden_size_list: List = [64, 128, 256],
            encoder_embedding_size: int = 512,
            twin_critic: bool = False,
            actor_head_hidden_size: int = 512,
            actor_head_layer_num: int = 1,
            critic_head_hidden_size: int = 512,
            critic_head_layer_num: int = 1,
            activation: Optional[nn.Module] = nn.ReLU(),
            norm_type: Optional[str] = None,
            **kwargs,
    ) -> None:
        super().__init__()

        self._act = nn.ReLU()
        self._obs_shape = obs_shape
        self._act_shape = action_shape
        self.twin_critic = twin_critic
        self.share_encoder = share_encoder
        if self.share_encoder:
            self.actor_encoder = self.critic_encoder = BEVSpeedConvEncoder(
                self._obs_shape, encoder_hidden_size_list, encoder_embedding_size, [3, 3, 3], [2, 2, 2]
            )
        else:
            self.actor_encoder = BEVSpeedConvEncoder(
                self._obs_shape, encoder_hidden_size_list, encoder_embedding_size, [3, 3, 3], [2, 2, 2]
            )
            self.critic_encoder = BEVSpeedConvEncoder(
                self._obs_shape, encoder_hidden_size_list, encoder_embedding_size, [3, 3, 3], [2, 2, 2]
            )

        self.actor = nn.Sequential(
            nn.Linear(encoder_embedding_size, actor_head_hidden_size), activation,
            ReparameterizationHead(
                actor_head_hidden_size,
                action_shape,
                actor_head_layer_num,
                sigma_type='conditioned',
                activation=activation,
                norm_type=norm_type
            )
        )
        self.twin_critic = twin_critic
        if self.twin_critic:
            if self.share_encoder:
                self._twin_encoder = self.actor_encoder
            else:
                self._twin_encoder = BEVSpeedConvEncoder(self._obs_shape, encoder_hidden_size_list, encoder_embedding_size, [3, 3, 3], [2, 2, 2])
            self.critic = nn.ModuleList()
            for _ in range(2):
                self.critic.append(
                    nn.Sequential(
                        nn.Linear(encoder_embedding_size + self._act_shape, critic_head_hidden_size), activation,
                        RegressionHead(
                            critic_head_hidden_size,
                            1,
                            critic_head_layer_num,
                            final_tanh=False,
                            activation=activation,
                            norm_type=norm_type
                        )
                    )
                )
        else:
            self.critic = nn.Sequential(
                nn.Linear(encoder_embedding_size + self._act_shape, critic_head_hidden_size), activation,
                RegressionHead(
                    critic_head_hidden_size,
                    1,
                    critic_head_layer_num,
                    final_tanh=False,
                    activation=activation,
                    norm_type=norm_type
                )
            )

    def forward(self, inputs, mode=None, **kwargs):
        self.mode = ['compute_actor', 'compute_critic']
        assert mode in self.mode, "not support forward mode: {}/{}".format(mode, self.mode)
        return getattr(self, mode)(inputs)

    def compute_critic(self, inputs: Dict[str, torch.Tensor]) -> Dict[str, torch.Tensor]:
        x0 = self.critic_encoder(inputs['obs'])
        x0 = torch.cat([x0, inputs['action']], dim=1)
        if self.twin_critic:
            x1 = self._twin_encoder(inputs['obs'])
            x1 = torch.cat([x1, inputs['action']], dim=1)
            x = [m(xi)['pred'] for m, xi in [(self.critic[0], x0), (self.critic[1], x1)]]
        else:
            x = self.critic(x0)['pred']
        return {'q_value': x}

    def compute_actor(self, inputs) -> Dict[str, torch.Tensor]:
        x = self.actor_encoder(inputs)
        x = self.actor(x)
        return {'logit': [x['mu'], x['sigma']]}


class PPORLModel(nn.Module):
    def __init__(
            self,
            obs_shape: Tuple = [5, 32, 32],
            action_shape: Union[int, Tuple] = 2,
            share_encoder: bool = True,
            continuous: bool = True,
            encoder_embedding_size: int = 512,
            encoder_hidden_size_list: List = [64, 128, 256],
            actor_head_hidden_size: int = 512,
            actor_head_layer_num: int = 1,
            critic_head_hidden_size: int = 512,
            critic_head_layer_num: int = 1,
            activation: Optional[nn.Module] = nn.ReLU(),
            norm_type: Optional[str] = None,
            sigma_type: Optional[str] = 'independent',
            bound_type: Optional[str] = None,
    ) -> None:
        super().__init__()
        self._obs_shape = obs_shape
        self._act_shape = action_shape
        self.share_encoder = share_encoder
        if self.share_encoder:
            self.encoder = BEVSpeedConvEncoder(
                self._obs_shape, encoder_hidden_size_list, encoder_embedding_size, [3, 3, 3], [2, 2, 2]
            )
        else:
            self.actor_encoder = BEVSpeedConvEncoder(
                self._obs_shape, encoder_hidden_size_list, encoder_embedding_size, [3, 3, 3], [2, 2, 2]
            )
            self.critic_encoder = BEVSpeedConvEncoder(
                self._obs_shape, encoder_hidden_size_list, encoder_embedding_size, [3, 3, 3], [2, 2, 2]
            )
        self.critic_head = RegressionHead(
            critic_head_hidden_size, 1, critic_head_layer_num, activation=activation, norm_type=norm_type
        )
        self.continuous = continuous
        if self.continuous:
            self.multi_head = False
            self.actor_head = ReparameterizationHead(
                actor_head_hidden_size,
                action_shape,
                actor_head_layer_num,
                sigma_type=sigma_type,
                activation=activation,
                norm_type=norm_type,
                bound_type=bound_type
            )
        else:
            multi_head = not isinstance(action_shape, int)
            self.multi_head = multi_head
            if multi_head:
                self.actor_head = MultiHead(
                    DiscreteHead,
                    actor_head_hidden_size,
                    action_shape,
                    layer_num=actor_head_layer_num,
                    activation=activation,
                    norm_type=norm_type
                )
            else:
                self.actor_head = DiscreteHead(
                    actor_head_hidden_size,
                    action_shape,
                    actor_head_layer_num,
                    activation=activation,
                    norm_type=norm_type
                )
        # for convenience of call some apis(such as: self.critic.parameters()), but may cause
        # misunderstanding when print(self)
        if self.share_encoder:
            self.actor = nn.ModuleList([self.encoder, self.actor_head])
            self.critic = nn.ModuleList([self.encoder, self.critic_head])
        else:
            self.actor = nn.ModuleList([self.actor_encoder, self.actor_head])
            self.critic = nn.ModuleList([self.critic_encoder, self.critic_head])

    def forward(self, inputs, mode=None, **kwargs):
        assert (mode in ['compute_actor_critic', 'compute_actor', 'compute_critic'])
        f = getattr(self, mode)
        return f(inputs, **kwargs)

    def compute_actor_critic(self, inputs) -> Dict[str, torch.Tensor]:
        if self.share_encoder:
            actor_embedding = critic_embedding = self.encoder(inputs)
        else:
            actor_embedding = self.actor_encoder(inputs)
            critic_embedding = self.critic_encoder(inputs)
        value = self.critic_head(critic_embedding)
        actor_output = self.actor_head(actor_embedding)
        if self.continuous:
            logit = [actor_output['mu'], actor_output['sigma']]
        else:
            logit = actor_output['logit']
        return {'logit': logit, 'value': value['pred']}

    def compute_actor(self, inputs: Dict) -> Dict:
        if self.share_encoder:
            x = self.encoder(inputs)
        else:
            x = self.actor_encoder(inputs)
        x = self.actor_head(x)
        if self.continuous:
            x = {'logit': [x['mu'], x['sigma']]}
        return x

    def compute_critic(self, inputs: Dict) -> Dict:
        if self.share_encoder:
            x = self.encoder(inputs)
        else:
            x = self.critic_encoder(inputs)
        x = self.critic_head(x)
        return {'value': x['pred']}
