from enum import Enum


class Gaits(Enum):

    TRI_GAIT = ("TRI_GAIT",
                [
                    # [amplitude, phase, duty_cycle and vertical_shift]
                    # phase = (phase + 0.5) % 1 -> invert direction
                    [
                        [0.05, 0.75, 0.5, -0.5],    # First leg, first joint (coxa)
                        [0.25, 0.5, 0.5, 0.5],      # First leg, second joint (femur)
                        [0.25, 0.5, 0.5, -0.25],    # First leg, third joint (tibia)
                    ],
                    [
                        [0.05, 0.25, 0.5, 0],
                        [0.25, 0, 0.5, 0.5],
                        [0.25, 0, 0.5, -0.25]
                    ],
                    [
                        [0.05, 0.75, 0.5, 0.5],
                        [0.25, 0.5, 0.5, 0.5],
                        [0.25, 0.5, 0.5, -0.25],
                    ],
                    [
                        [0.05, 0.75, 0.5, -0.5],
                        [0.25, 0.5, 0.5, -0.5],
                        [0.25, 0, 0.5, -0.25]
                    ],
                    [
                        [0.05, 0.25, 0.5, 0],
                        [0.25, 0, 0.5, -0.5],
                        [0.25, 0.5, 0.5, -0.25],
                    ],
                    [
                        [0.05, 0.75, 0.5, 0.5],
                        [0.25, 0.5, 0.5, -0.5],
                        [0.25, 0, 0.5, -0.25]
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
        for item in Gaits:
            if item.label == label:
                return item
        raise ValueError(f"No enum member found with label `{label}`.")


if __name__ == '__main__':

    gait = Gaits.TRI_GAIT
    print(f"Gait: {gait.label}")
    print(f"Data: {gait.data}")
