import torch
import torch.nn as nn
from typing import Dict, Optional, Tuple, List, Union

from ding.torch_utils import MLP


class BEVSpeedConvEncoder(nn.Module):
    """
    Convolutional encoder of Bird-eye View image and speed input. It takes a BeV image and a speed scalar as input.
    The BeV image is encoded by a convolutional encoder, to get a embedding feature which is half size of the
    embedding length. Then the speed value is repeated for half embedding length time, and concated to the above
    feature to get a final feature.

    :Arguments:
        - obs_shape (Tuple): BeV image shape.
        - hidden_dim_list (List): Conv encoder hidden layer dimension list.
        - embedding_size (int): Embedding feature dimensions.
        - kernel_size (List, optional): Conv kernel size for each layer. Defaults to [8, 4, 3].
        - stride (List, optional): Conv stride for each layer. Defaults to [4, 2, 1].
    """

    def __init__(
            self,
            obs_shape: Tuple,
            hidden_dim_list: List,
            embedding_size: int,
            kernel_size: List = [8, 4, 3],
            stride: List = [4, 2, 1],
    ) -> None:
        super().__init__()
        assert len(kernel_size) == len(stride), (kernel_size, stride)
        self._obs_shape = obs_shape
        self._embedding_size = embedding_size

        self._relu = nn.ReLU()
        layers = []
        input_dim = obs_shape[0]
        for i in range(len(hidden_dim_list)):
            layers.append(nn.Conv2d(input_dim, hidden_dim_list[i], kernel_size[i], stride[i]))
            layers.append(self._relu)
            input_dim = hidden_dim_list[i]
        layers.append(nn.Flatten())
        self._model = nn.Sequential(*layers)
        flatten_size = self._get_flatten_size()
        self._mid = nn.Linear(flatten_size, self._embedding_size // 2)

    def _get_flatten_size(self) -> int:
        test_data = torch.randn(1, *self._obs_shape)
        with torch.no_grad():
            output = self._model(test_data)
        return output.shape[1]

    def forward(self, data: Dict) -> torch.Tensor:
        """
        Forward computation of encoder

        :Arguments:
            - data (Dict): Input data, must contain 'birdview' and 'speed'

        :Returns:
            torch.Tensor: Embedding feature.
        """
        image = data['birdview'].permute(0, 3, 1, 2)
        speed = data['speed']
        x = self._model(image)
        x = self._mid(x)
        speed_embedding_size = self._embedding_size - self._embedding_size // 2
        speed_vec = torch.unsqueeze(speed, 1).repeat(1, speed_embedding_size)
        h = torch.cat((x, speed_vec), dim=1)
        return h
