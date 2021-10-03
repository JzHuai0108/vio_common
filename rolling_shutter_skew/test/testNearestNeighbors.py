import kNearestNeighbor

import numpy as np
import random


def testKNearestNeighbor():
    l = 25
    trainData = np.random.randint(0,100,(l,2)).astype(np.float32)
    k = 3
    result, _ = kNearestNeighbor.nearestNeighbors(trainData, k)

    index = random.randrange(0, l)
    dist = []
    for row in trainData:
        dist.append(np.linalg.norm(row - trainData[index, :]))
    order = np.argsort(dist)

    for i in range(k):
        assert result[index][i] == order[i+1]
