import largestComponent


def testLargestComponent():
    # Create a graph given in the above diagram
    # 5 vertices numbered from 0 to 4
    g = largestComponent.Graph(5)
    g.addEdge(1, 0)
    g.addEdge(2, 3)
    g.addEdge(3, 4)
    g.addEdge(0, 1)
    g.addEdge(3, 2)
    g.addEdge(4, 3)
    cc = g.connectedComponents()
    assert cc == [[0, 1], [2, 3, 4]]
