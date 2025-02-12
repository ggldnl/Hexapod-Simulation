from enum import Enum

import numpy as np


class Gait(Enum):

    TRI_GAIT = ("TRI_GAIT",
                [
                    [
                        [1, 0, 0.5, np.pi / 4],
                        [0.25, 0.25, 0.5, np.pi / 4],
                        [1, 0, 0.5, np.pi / 4],
                    ],
                    [
                        [1, 0.5, 0.5, np.pi / 4],
                        [0.25, 0.75, 0.5, np.pi / 4],
                        [1, 0.5, 0.5, np.pi / 4]
                    ],
                    [
                        [1, 0, 0.5, np.pi / 4],
                        [0.25, 0.25, 0.5, np.pi / 4],
                        [1, 0, 0.5, np.pi / 4]
                    ],
                    [
                        [1, 0, 0.5, np.pi / 4],
                        [0.25, 0.75, 0.5, np.pi / 4],
                        [1, 0, 0.5, np.pi / 4]
                    ],
                    [
                        [1, 0.5, 0.5, np.pi / 4],
                        [0.25, 0.25, 0.5, np.pi / 4],
                        [1, 0.5, 0.5, np.pi / 4]
                    ],
                    [
                        [1, 0, 0.5, np.pi / 4],
                        [0.25, 0.75, 0.5, np.pi / 4],
                        [1, 0, 0.5, np.pi / 4]
                    ]
                ]
    )

    def __init__(self, label, data):
        self._label = label
        self._data = data

    @property
    def label(self):
        return self._label

    @property
    def data(self):
        return self._data

    def __getitem__(self, label):
        for item in Gait:
            if item.label == label:
                return item
        raise ValueError(f"No enum member found with label `{label}`.")


if __name__ == '__main__':

    gait = Gait.TRI_GAIT
    print(f"Gait: {gait.label}")
    print(f"Data: {gait.data}")
