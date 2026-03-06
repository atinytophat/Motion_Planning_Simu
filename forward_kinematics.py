import numpy as np


def modDH(a, alpha, d, theta):
    T0 = np.array(
        [
            [np.cos(np.radians(theta[0])), -np.sin(np.radians(theta[0])), 0, a[0]],
            [
                np.sin(np.radians(theta[0])) * np.cos(np.radians(alpha[0])),
                np.cos(np.radians(theta[0])) * np.cos(np.radians(alpha[0])),
                -np.sin(np.radians(alpha[0])),
                d[0] * -np.sin(np.radians(alpha[0])),
            ],
            [
                np.sin(np.radians(theta[0])) * np.sin(np.radians(alpha[0])),
                np.cos(np.radians(theta[0])) * np.sin(np.radians(alpha[0])),
                np.cos(np.radians(alpha[0])),
                d[0] * np.cos(np.radians(alpha[0])),
            ],
            [0, 0, 0, 1],
        ]
    )

    T = T0

    if len(a) >= 2:
        for i in range(1, len(a)):
            Ti = np.array(
                [
                    [
                        np.cos(np.radians(theta[i])),
                        -np.sin(np.radians(theta[i])),
                        0,
                        a[i],
                    ],
                    [
                        np.sin(np.radians(theta[i])) * np.cos(np.radians(alpha[i])),
                        np.cos(np.radians(theta[i])) * np.cos(np.radians(alpha[i])),
                        -np.sin(np.radians(alpha[i])),
                        d[i] * -np.sin(np.radians(alpha[i])),
                    ],
                    [
                        np.sin(np.radians(theta[i])) * np.sin(np.radians(alpha[i])),
                        np.cos(np.radians(theta[i])) * np.sin(np.radians(alpha[i])),
                        np.cos(np.radians(alpha[i])),
                        d[i] * np.cos(np.radians(alpha[i])),
                    ],
                    [0, 0, 0, 1],
                ]
            )
            T = T @ Ti

    return T