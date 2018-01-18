tree = {1:[2, 3], 2:[4, 5], 3:[6,7], 4:[8,9],
        5:[10,11], 6:[12,13], 7:[14,15]}

#pretty much does bfs but saves the path every time in a queue, so I have a queue of queues

def bfs(tree, start, goal):
    # queue of paths
    queue = [];
    # push the first path into the queue
    queue.append([start])
    while queue:
        # get the first path from the queue
        path = queue.pop(0);
   
        node = path[-1];
        # path found
        if node == goal:
            return path;
        for nieghbor in tree.get(node, []):
            newPath = list(path);
            newPath.append(nieghbor);
            queue.append(newPath);

print bfs(tree, 1, 11)


