
function DepthFirstSearch(g, start, list){
  var visited;
  var ll;

  visited = CreateBooleanArray(g.nodes.length, false);
  ll = CreateLinkedListNumbers();

  DepthFirstSearchRecursive(g, g.nodes[start], start, visited, ll);

  list.numberArray = LinkedListNumbersToArray(ll);
  FreeLinkedListNumbers(ll);
}
function DepthFirstSearchRecursive(g, node, nodeNr, visited, list){
  var i;
  var e;

  visited[nodeNr] = true;

  LinkedListAddNumber(list, nodeNr);

  for(i = 0; i < node.edge.length; i = i + 1){
    e = node.edge[i];
    if( !visited[e.nodeNr] ){
      DepthFirstSearchRecursive(g, g.nodes[e.nodeNr], e.nodeNr, visited, list);
    }
  }
}
function BreadthFirstSearch(g, start, list){
  var visited;
  var i, front, v, length;
  var e;
  var n;
  var da;

  da = CreateDynamicArrayNumbers();
  visited = CreateBooleanArray(g.nodes.length, false);
  length = 0;
  front = 0;

  visited[start] = true;

  DynamicArrayAddNumber(da, start);
  length = length + 1;

  for(; front != length; ){
    v = DynamicArrayNumbersIndex(da, front);
    front = front + 1;

    n = g.nodes[v];

    for(i = 0; i < n.edge.length; i = i + 1){
      e = n.edge[i];
      if( !visited[e.nodeNr] ){
        visited[e.nodeNr] = true;

        DynamicArrayAddNumber(da, e.nodeNr);
        length = length + 1;
      }
    }
  }

  list.numberArray = DynamicArrayNumbersToArray(da);
  FreeDynamicArrayNumbers(da);
}
function PrimsAlgorithmNoQueue(g, forest){
  var valid, found, minimumSet;
  var inMST;
  var i, j, root, minimum, minimumTarget, minimumSource, nodesCompleted;
  var node;
  var edge;
  var linkedListTrees;
  var roots;

  valid = DirectedGraphIsValid(g) && IsUndirected(g);

  if(valid){
    inMST = CreateBooleanArray(g.nodes.length, false);
    nodesCompleted = 0;
    linkedListTrees = CreateLinkedListNumbersArray(g.nodes.length);
    roots = CreateLinkedListNumbers();

    for(; nodesCompleted < g.nodes.length; ){

      /* Find a node not in an MST */
      found = false;
      root = 0;
      for(i = 0; i < g.nodes.length &&  !found ; i = i + 1){
        if( !inMST[i] ){
          root = i;
          found = true;
        }
      }

      LinkedListAddNumber(roots, root);

      inMST[root] = true;
      nodesCompleted = nodesCompleted + 1;

      found = true;
      for(; found; ){
        /* Find minimum edge going out from existing tree. */
        minimum = 0;
        minimumSet = false;
        minimumTarget = 0;
        minimumSource = 0;
        for(i = 0; i < g.nodes.length; i = i + 1){
          if(inMST[i]){
            node = g.nodes[i];
            for(j = 0; j < node.edge.length; j = j + 1){
              edge = node.edge[j];
              if( !inMST[edge.nodeNr] ){
                if( !minimumSet ){
                  minimum = edge.weight;
                  minimumTarget = edge.nodeNr;
                  minimumSource = i;
                  minimumSet = true;
                }else if(edge.weight < minimum){
                  minimum = edge.weight;
                  minimumTarget = edge.nodeNr;
                  minimumSource = i;
                }
              }
            }
          }
        }

        /* Add edge to tree. */
        if(minimumSet){
          LinkedListAddNumber(linkedListTrees[minimumSource], minimumTarget);
          inMST[minimumTarget] = true;
          nodesCompleted = nodesCompleted + 1;
          found = true;
        }else{
          found = false;
        }
      }
    }

    ConvertLinkedListTreesToForest(forest, roots, linkedListTrees);

    /* Free memory. */
    delete(inMST);
    FreeLinkedListNumbersArray(linkedListTrees);
  }

  return valid;
}
function PrimsAlgorithm(g, forest){
  var valid, found, minimumSet, empty;
  var inMST, minimumEdgeSet;
  var i, root, minimumTarget, minimumSource, nodesCompleted;
  var minimumEdges, minimumSources;
  var q;
  var targetReference, weightReference;
  var linkedListTrees;
  var roots;

  valid = DirectedGraphIsValid(g) && IsUndirected(g);

  if(valid){
    q = CreatePriorityQueueBTNumKeyValue();
    targetReference = {};
    weightReference = {};
    inMST = CreateBooleanArray(g.nodes.length, false);
    minimumEdgeSet = CreateBooleanArray(g.nodes.length, false);
    minimumEdges = CreateNumberArray(g.nodes.length, 0);
    minimumSources = CreateNumberArray(g.nodes.length, 0);
    linkedListTrees = CreateLinkedListNumbersArray(g.nodes.length);
    roots = CreateLinkedListNumbers();
    nodesCompleted = 0;

    for(; nodesCompleted < g.nodes.length; ){
      /* Find a node not in an MST */
      found = false;
      root = 0;
      for(i = 0; i < g.nodes.length &&  !found ; i = i + 1){
        if( !inMST[i] ){
          root = i;
          found = true;
        }
      }

      /* Record tree root. */
      LinkedListAddNumber(roots, root);
      inMST[root] = true;
      nodesCompleted = nodesCompleted + 1;

      /* Add all outgoing edges to priority queue */
      AddOutgoingEdgesToPriorityQueue(g.nodes[root], minimumEdgeSet, root, minimumEdges, minimumSources, q);

      /* Expand tree one vertex at a time. */
      found = true;
      for(; found; ){
        /* Find minimum edge going out from existing tree using queue. */
        minimumSet = false;
        empty = false;
        for(;  !minimumSet  &&  !empty ; ){
          empty =  !PopPriorityQueueBTNumKeyValue(q, weightReference, targetReference) ;
          if( !empty  &&  !inMST[targetReference.numberValue] ){
            minimumSet = true;
          }
        }

        if(minimumSet){
          /* Add edge to tree. */
          minimumTarget = targetReference.numberValue;
          minimumSource = minimumSources[minimumTarget];

          LinkedListAddNumber(linkedListTrees[minimumSource], minimumTarget);
          inMST[minimumTarget] = true;
          nodesCompleted = nodesCompleted + 1;
          found = true;

          /* Add all outgoing edges to priority queue. */
          AddOutgoingEdgesToPriorityQueue(g.nodes[minimumTarget], minimumEdgeSet, minimumTarget, minimumEdges, minimumSources, q);
        }else{
          found = false;
        }
      }
    }

    ConvertLinkedListTreesToForest(forest, roots, linkedListTrees);

    /* Free memory. */
    FreePriorityQueueBTNumKeyValue(q);
    delete(targetReference);
    delete(weightReference);
    delete(inMST);
    delete(minimumEdgeSet);
    FreeLinkedListNumbersArray(linkedListTrees);
    delete(minimumEdges);
    delete(minimumSources);
  }

  return valid;
}
function AddOutgoingEdgesToPriorityQueue(node, minimumEdgeSet, source, minimumEdges, minimumSources, q){
  var i, target;
  var edge;

  for(i = 0; i < node.edge.length; i = i + 1){
    edge = node.edge[i];
    target = edge.nodeNr;
    InsertIntoPriorityQueueBTNumKeyValue(q, 1/edge.weight, target);
    if( !minimumEdgeSet[target] ){
      minimumEdges[target] = edge.weight;
      minimumSources[target] = source;
      minimumEdgeSet[target] = true;
    }else if(minimumEdges[target] > edge.weight){
      minimumEdges[target] = edge.weight;
      minimumSources[target] = source;
    }
  }
}
function KruskalsAlgorithm(g, forest){
  var valid;
  var q;
  var node;
  var edge;
  var i, j, edgeNr, source, target, replace, replaceWith, candidate, trees, treeNr;
  var sources, targets;
  var edges;
  var memberOfTree;
  var roots;
  var edgeNrReference, weightReference;
  var tree;

  valid = DirectedGraphIsValid(g) && IsUndirected(g);

  if(valid){
    sources = CreateDynamicArrayNumbers();
    targets = CreateDynamicArrayNumbers();
    edges = CreateDynamicArrayNumbers();
    edgeNrReference = {};
    weightReference = {};
    roots = CreateBooleanArray(g.nodes.length, false);
    memberOfTree = [];
    memberOfTree.length = g.nodes.length;
    for(i = 0; i < g.nodes.length; i = i + 1){
      memberOfTree[i] = i;
    }

    q = CreatePriorityQueueBTNumKeyValue();

    /* Add all edges to a priority queue. */
    edgeNr = 0;
    for(i = 0; i < g.nodes.length; i = i + 1){
      node = g.nodes[i];
      for(j = 0; j < node.edge.length; j = j + 1){
        edge = node.edge[j];
        InsertIntoPriorityQueueBTNumKeyValue(q, 1/edge.weight, edgeNr);
        DynamicArrayAddNumber(sources, i);
        DynamicArrayAddNumber(targets, edge.nodeNr);

        edgeNr = edgeNr + 1;
      }
    }

    for(;  !IsEmptyPriorityQueueBTNumKeyValue(q) ; ){
      PopPriorityQueueBTNumKeyValue(q, weightReference, edgeNrReference);

      source = DynamicArrayNumbersIndex(sources, edgeNrReference.numberValue);
      target = DynamicArrayNumbersIndex(targets, edgeNrReference.numberValue);

      if(memberOfTree[source] != memberOfTree[target]){
        replace = memberOfTree[target];
        replaceWith = memberOfTree[source];

        for(i = 0; i < g.nodes.length; i = i + 1){
          if(memberOfTree[i] == replace){
            memberOfTree[i] = replaceWith;
          }
        }

        DynamicArrayAddNumber(edges, edgeNrReference.numberValue);
      }
    }

    /* Built forest. */
    trees = 0;
    for(i = 0; i < g.nodes.length; i = i + 1){
      candidate = memberOfTree[i];
      if( !roots[candidate] ){
        trees = trees + 1;
        roots[candidate] = true;
      }
    }
    forest.trees = [];
    forest.trees.length = trees;
    treeNr = 0;
    for(i = 0; i < g.nodes.length; i = i + 1){
      if(roots[i]){
        tree = CreateTreeFromEdgeCollection(i, i, edges, sources, targets);

        forest.trees[treeNr] = tree;
        treeNr = treeNr + 1;
      }
    }

    /* Free memory. */
    FreePriorityQueueBTNumKeyValue(q);
    FreeDynamicArrayNumbers(sources);
    FreeDynamicArrayNumbers(targets);
    delete(edgeNrReference);
    delete(weightReference);
  }

  return valid;
}
function CreateTreeFromEdgeCollection(root, parent, edges, sources, targets){
  var tree;
  var i, edgeNr, source, target, size;
  var branches;
  var node;

  tree = {};
  tree.label = root;
  branches = CreateLinkedListNumbers();

  size = 0;
  for(i = 0; i < edges.length; i = i + 1){
    edgeNr = DynamicArrayNumbersIndex(edges, i);

    source = DynamicArrayNumbersIndex(sources, edgeNr);
    target = DynamicArrayNumbersIndex(targets, edgeNr);

    if(source == root && target != parent){
      LinkedListAddNumber(branches, target);
      size = size + 1;
    }else if(target == root && source != parent){
      LinkedListAddNumber(branches, source);
      size = size + 1;
    }
  }

  tree.branches = [];
  tree.branches.length = size;
  node = branches.first;
  for(i = 0; i < size; i = i + 1){
    tree.branches[i] = CreateTreeFromEdgeCollection(node.value, root, edges, sources, targets);

    node = node.next;
  }

  return tree;
}
function DijkstrasAlgorithm(g, src, dist, distSet, prev){
  var nodeDone;
  var i, j, v, nodes, u, edges, alt;
  var edge;

  nodes = g.nodes.length;

  distSet.booleanArray = CreateBooleanArray(nodes, false);
  nodeDone = CreateBooleanArray(nodes, false);
  dist.numberArray = CreateNumberArray(nodes, 0);
  prev.numberArray = CreateNumberArray(nodes, 0);

  dist.numberArray[src] = 0;
  distSet.booleanArray[src] = true;

  for(i = 0; i < nodes; i = i + 1){
    /* Get node with lowest distance */
    u = ListFindLowestSetAndIncluded(dist.numberArray, distSet.booleanArray, nodeDone);

    /* Mark node as done */
    nodeDone[u] = true;

    edges = GetEdgesForNodeFromDirectedGraph(g, u);
    for(j = 0; j < edges; j = j + 1){
      edge = GetEdgeFromDirectedGraph(g, u, j);

      if( !nodeDone[edge.nodeNr] ){
        v = edge.nodeNr;
        alt = dist.numberArray[u] + edge.weight;
        if( !distSet.booleanArray[v] ){
          dist.numberArray[v] = alt;
          distSet.booleanArray[v] = true;
          prev.numberArray[v] = u;
        }else if(alt < dist.numberArray[v]){
          dist.numberArray[v] = alt;
          prev.numberArray[v] = u;
        }
      }
    }
  }

  delete(distSet);
  delete(nodeDone);
}
function ListFindLowestSetAndIncluded(list, set, exclude){
  var i, nodes, lowest, u;
  var lowestSet;

  nodes = list.length;
  lowest = 0;
  u = 0;

  lowestSet = false;
  for(i = 0; i < nodes; i = i + 1){
    if( !exclude[i]  && set[i]){
      if( !lowestSet ){
        lowest = list[i];
        u = i;
        lowestSet = true;
      }else if(list[i] < lowest){
        lowest = list[i];
        u = i;
      }
    }
  }

  return u;
}
function DijkstrasAlgorithmDestinationOnly(g, src, dest, path, distance){
  var distances, previous;
  var distanceSet;
  var found;

  distances = {};
  previous = {};
  distanceSet = {};

  DijkstrasAlgorithm(g, src, distances, distanceSet, previous);

  found = distanceSet.booleanArray[dest];

  if(found){
    distance.numberValue = distances.numberArray[dest];

    ExtractForwardPathFromReverseList(src, dest, previous, path);
  }

  delete(distances);
  delete(previous);
  delete(distanceSet);

  return found;
}
function ExtractForwardPathFromReverseList(src, dest, previous, path){
  var next, length, i;

  next = dest;
  for(length = 1; next != src; length = length + 1){
    next = previous.numberArray[next];
  }

  path.numberArray = CreateNumberArray(length, 0);

  next = dest;
  for(i = 0; i < length; i = i + 1){
    path.numberArray[length - i - 1] = next;
    next = previous.numberArray[next];
  }
}
function BellmanFordAlgorithm(g, src, dist, distSet, prev){
  var nodeDone;
  var i, j, v, nodes, u, edges, w;
  var edge;
  var success;

  nodes = g.nodes.length;

  distSet.booleanArray = CreateBooleanArray(nodes, false);
  nodeDone = CreateBooleanArray(nodes, false);
  dist.numberArray = CreateNumberArray(nodes, 0);
  prev.numberArray = CreateNumberArray(nodes, 0);

  dist.numberArray[src] = 0;
  distSet.booleanArray[src] = true;

  for(i = 0; i < nodes - 1; i = i + 1){
    for(u = 0; u < nodes; u = u + 1){
      edges = GetEdgesForNodeFromDirectedGraph(g, u);
      for(j = 0; j < edges; j = j + 1){
        edge = GetEdgeFromDirectedGraph(g, u, j);

        v = edge.nodeNr;
        w = edge.weight;

        if(distSet.booleanArray[u]){
          if( !distSet.booleanArray[v] ){
            dist.numberArray[v] = dist.numberArray[u] + w;
            distSet.booleanArray[v] = true;
            prev.numberArray[v] = u;
          }else if(dist.numberArray[u] + w < dist.numberArray[v]){
            dist.numberArray[v] = dist.numberArray[u] + w;
            prev.numberArray[v] = u;
          }
        }
      }
    }
  }

  success = true;
  for(u = 0; u < nodes; u = u + 1){
    edges = GetEdgesForNodeFromDirectedGraph(g, u);
    for(j = 0; j < edges; j = j + 1){
      edge = GetEdgeFromDirectedGraph(g, u, j);

      v = edge.nodeNr;
      w = edge.weight;

      if(dist.numberArray[u] + w < dist.numberArray[v]){
        success = false;
      }
    }
  }

  delete(distSet);
  delete(nodeDone);

  return success;
}
function BellmanFordAlgorithmDestinationOnly(g, src, dest, path, distance){
  var distances, previous;
  var distanceSet;
  var found;

  distances = {};
  previous = {};
  distanceSet = {};

  found = BellmanFordAlgorithm(g, src, distances, distanceSet, previous);

  if(found){
    found = distanceSet.booleanArray[dest];

    if(found){
      distance.numberValue = distances.numberArray[dest];

      ExtractForwardPathFromReverseList(src, dest, previous, path);
    }
  }

  delete(distances);
  delete(previous);
  delete(distanceSet);

  return found;
}
function FloydWarshallAlgorithm(g, distances){
  var u, v, k, i, j;
  var n;
  var e;
  var t, ij, ik, kj;
  var success;

  success = true;

  for(u = 0; u < g.nodes.length; u = u + 1){
    n = g.nodes[u];

    for(j = 0; j < n.edge.length; j = j + 1){
      e = n.edge[j];
      v = e.nodeNr;

      t = distances.from[u].to[v];
      t.length = e.weight;
      t.lengthSet = true;
      t.next = v;
    }
  }

  for(v = 0; v < g.nodes.length; v = v + 1){
    t = distances.from[v].to[v];
    t.length = 0;
    t.lengthSet = true;
    t.next = v;
  }

  for(k = 0; k < g.nodes.length && success; k = k + 1){
    for(i = 0; i < g.nodes.length && success; i = i + 1){
      for(j = 0; j < g.nodes.length && success; j = j + 1){
        ij = distances.from[i].to[j];
        ik = distances.from[i].to[k];
        kj = distances.from[k].to[j];

        if( !ij.lengthSet  && ik.lengthSet && kj.lengthSet){
          ij.length = ik.length + kj.length;
          ij.lengthSet = true;
          ij.next = ik.next;
        }else if(ij.lengthSet && ik.lengthSet && kj.lengthSet){
          if(ij.length > ik.length + kj.length){
            ij.length = ik.length + kj.length;
            ij.next = ik.next;
          }
        }

        if(i == j){
          if(ij.length < 0){
            success = false;
          }
        }
      }
    }
  }

  return success;
}
function CreateDistancesFloydWarshallAlgorithm(nodes){
  var distances;
  var i, j;

  distances = {};
  distances.from = [];
  distances.from.length = nodes;
  for(i = 0; i < distances.from.length; i = i + 1){
    distances.from[i] = {};
    distances.from[i].to = [];
    distances.from[i].to.length = distances.from.length;
    for(j = 0; j < distances.from.length; j = j + 1){
      distances.from[i].to[j] = {};
      distances.from[i].to[j].length = 0;
      distances.from[i].to[j].lengthSet = false;
      distances.from[i].to[j].next = 0;
    }
  }

  return distances;
}
function GetPathFromDistances(distances, u, v){
  var path;
  var length, next, i;
  var t;

  t = distances.from[u].to[v];
  if(t.lengthSet){
    /* count */
    length = 1;
    next = u;
    for(; next != v; ){
      next = distances.from[next].to[v].next;
      length = length + 1;
    }

    path = [];
    path.length = length;

    /* set */
    next = u;
    for(i = 0; i < length; i = i + 1){
      path[i] = next;
      next = distances.from[next].to[v].next;
    }
  }else{
    path = [];
    path.length = 0;
  }

  return path;
}
function CreateEdge(nodeNr, weight){
  var e;

  e = {};

  e.nodeNr = nodeNr;
  e.weight = weight;

  return e;
}
function DirectedGraphIsValid(g){
  var valid;
  var i, j;
  var node;
  var edge;

  valid = true;

  for(i = 0; i < g.nodes.length; i = i + 1){
    node = g.nodes[i];
    for(j = 0; j < node.edge.length; j = j + 1){
      edge = node.edge[j];
      if(IsInteger(edge.nodeNr)){
        if(edge.nodeNr >= 0 && edge.nodeNr < g.nodes.length){
        }else{
          valid = false;
        }
      }else{
        valid = false;
      }
    }
  }

  return valid;
}
function DirectedGraphContainsCycleDFS(g){
  var incoming;
  var i, zeroIncomming;
  var cycle;

  cycle = false;
  incoming = DirectedGraphCountIncomingEdges(g);

  zeroIncomming = 0;
  for(i = 0; i < g.nodes.length &&  !cycle ; i = i + 1){
    if(incoming[i] == 0){
      zeroIncomming = zeroIncomming + 1;

      cycle = DirectedGraphContainsCycleFromNodeDFS(g, i);
    }
  }

  delete(incoming);

  if(g.nodes.length > 0 && zeroIncomming == 0){
    cycle = true;
  }

  return cycle;
}
function DirectedGraphCountIncomingEdges(g){
  var incoming;
  var i, j;
  var node;
  var e;

  incoming = CreateNumberArray(g.nodes.length, 0);

  for(i = 0; i < g.nodes.length; i = i + 1){
    node = g.nodes[i];
    for(j = 0; j < node.edge.length; j = j + 1){
      e = node.edge[j];
      incoming[e.nodeNr] = incoming[e.nodeNr] + 1;
    }
  }

  return incoming;
}
function DirectedGraphContainsCycleFromNodeDFS(g, nodeNr){
  var visited;
  var cycle;

  visited = CreateBooleanArray(g.nodes.length, false);

  cycle = DirectedGraphContainsCycleFromNodeDFSRecursive(g, nodeNr, visited);

  delete(visited);

  return cycle;
}
function DirectedGraphContainsCycleFromNodeDFSRecursive(g, nodeNr, visited){
  var i;
  var e;
  var cycle;
  var node;

  cycle = false;
  node = g.nodes[nodeNr];

  for(i = 0; i < node.edge.length &&  !cycle ; i = i + 1){
    e = node.edge[i];
    if(visited[e.nodeNr]){
      cycle = true;
    }else{
      visited[e.nodeNr] = true;
      cycle = DirectedGraphContainsCycleFromNodeDFSRecursive(g, e.nodeNr, visited);
      visited[e.nodeNr] = false;
    }
  }

  return cycle;
}
function DirectedGraphCountCyclesDFS(g){
  var incoming;
  var i, zeroIncoming, cycleCount;

  cycleCount = 0;
  incoming = DirectedGraphCountIncomingEdges(g);

  zeroIncoming = 0;
  for(i = 0; i < g.nodes.length; i = i + 1){
    if(incoming[i] == 0){
      zeroIncoming = zeroIncoming + 1;

      cycleCount = cycleCount + DirectedGraphCountCyclesFromNodeDFS(g, i);
    }
  }

  if(g.nodes.length > 0 && zeroIncoming == 0){
    cycleCount = cycleCount + DirectedGraphCountCyclesFromNodeDFS(g, 0);
  }

  delete(incoming);

  return cycleCount;
}
function DirectedGraphCountCyclesFromNodeDFS(g, nodeNr){
  var color;
  var cycleCount;

  color = CreateNumberArray(g.nodes.length, 0);

  cycleCount = DirectedGraphCountCyclesFromNodeDFSRecursive(g, nodeNr, color);

  delete(color);

  return cycleCount;
}
function DirectedGraphCountCyclesFromNodeDFSRecursive(g, nodeNr, color){
  var i, cycleCount;
  var e;
  var node;

  cycleCount = 0;
  node = g.nodes[nodeNr];

  color[nodeNr] = 1;

  for(i = 0; i < node.edge.length; i = i + 1){
    e = node.edge[i];

    if(color[e.nodeNr] != 2){
      if(color[e.nodeNr] == 1){
        cycleCount = cycleCount + 1;
      }else{
        cycleCount = cycleCount + DirectedGraphCountCyclesFromNodeDFSRecursive(g, e.nodeNr, color);
      }
    }
  }

  color[nodeNr] = 2;

  return cycleCount;
}
function DirectedGraphGetCyclesDFS(g){
  var cycleCount;
  var cycles;
  var incoming;
  var i, zeroIncoming;
  var cycleNumber;

  cycleNumber = CreateNumberReference(0);
  cycleCount = DirectedGraphCountCyclesDFS(g);

  cycles = [];
  cycles.length = cycleCount;

  incoming = DirectedGraphCountIncomingEdges(g);

  zeroIncoming = 0;
  for(i = 0; i < g.nodes.length; i = i + 1){
    if(incoming[i] == 0){
      zeroIncoming = zeroIncoming + 1;

      DirectedGraphGetCyclesFromNodeDFS(g, i, cycles, cycleNumber);
    }
  }

  if(g.nodes.length > 0 && zeroIncoming == 0){
    DirectedGraphGetCyclesFromNodeDFS(g, 0, cycles, cycleNumber);
  }

  delete(incoming);

  return cycles;
}
function DirectedGraphGetCyclesFromNodeDFS(g, nodeNr, cycles, cycleNumber){
  var color, cycleMark;
  var previous;
  var previousLength;

  color = CreateNumberArray(g.nodes.length, 0);
  cycleMark = CreateNumberArray(g.nodes.length, 0);
  previous = CreateNumberArray(g.nodes.length, 0);
  previousLength = 0;

  DirectedGraphGetCyclesFromNodeDFSRecursive(g, nodeNr, cycleNumber, color, cycles, previous, previousLength);

  delete(color);
  delete(cycleMark);
}
function DirectedGraphGetCyclesFromNodeDFSRecursive(g, nodeNr, cycleNumber, color, cycles, previous, previousLength){
  var i, j, current, cycleLength, next;
  var e;
  var node;
  var done;

  node = g.nodes[nodeNr];

  color[nodeNr] = 1;

  previous[previousLength] = nodeNr;

  for(i = 0; i < node.edge.length; i = i + 1){
    e = node.edge[i];
    if(color[e.nodeNr] != 2){
      if(color[e.nodeNr] == 1){
        /* Get cycle length */
        cycleLength = 0;
        done = false;
        current = previousLength;
        for(;  !done ; ){
          cycleLength = cycleLength + 1;
          if(previous[current] == e.nodeNr){
            done = true;
          }
          current = current - 1;
        }

        /* Get cycle in order */
        cycles[cycleNumber.numberValue] = {};
        cycles[cycleNumber.numberValue].nodeNrs = [];
        cycles[cycleNumber.numberValue].nodeNrs.length = cycleLength;
        for(j = 0; j < cycleLength; j = j + 1){
          next = previousLength - cycleLength + 1 + j;
          cycles[cycleNumber.numberValue].nodeNrs[j] = previous[next];
        }

        cycleNumber.numberValue = cycleNumber.numberValue + 1;
      }else{
        DirectedGraphGetCyclesFromNodeDFSRecursive(g, e.nodeNr, cycleNumber, color, cycles, previous, previousLength + 1);
      }
    }
  }

  color[nodeNr] = 2;
}
function CreateDirectedGraph(nodes){
  var directedGraph;
  var i;

  directedGraph = {};
  directedGraph.nodes = [];
  directedGraph.nodes.length = nodes;

  for(i = 0; i < nodes; i = i + 1){
    directedGraph.nodes[i] = {};
  }

  return directedGraph;
}
function CreateDirectedGraphFromMatrixForm(m){
  var g;
  var nodes, i, j, order, edgeValue, edgeNr;

  nodes = GetNodesFromDirectedGraphMatrix(m);

  g = CreateDirectedGraph(nodes);

  for(i = 0; i < nodes; i = i + 1){
    order = GetNodeOrderFromMatrixForm(m, i);
    g.nodes[i].edge = [];
    g.nodes[i].edge.length = order;
    edgeNr = 0;
    for(j = 0; j < nodes; j = j + 1){
      edgeValue = GetDirectedGraphMatrixEdge(m, i, j);
      if(edgeValue != 0){
        g.nodes[i].edge[edgeNr] = CreateEdge(j, edgeValue);
        edgeNr = edgeNr + 1;
      }
    }
  }

  return g;
}
function GetDirectedGraphMatrixEdge(m, nodeNr, edgeNr){
  return m.c[nodeNr].r[edgeNr];
}
function GetNodeOrderFromMatrixForm(m, nodeNr){
  var nodes, i, order;

  nodes = GetNodesFromDirectedGraphMatrix(m);

  order = 0;
  for(i = 0; i < nodes; i = i + 1){
    order = order + m.c[nodeNr].r[i];
  }

  return order;
}
function GetNodesFromDirectedGraphMatrix(m){
  return m.c.length;
}
function CreateDirectedGraphMatrixFromListForm(g){
  var m;
  var i, j, nodes;
  var node;
  var edge;

  nodes = g.nodes.length;
  m = CreateDirectedGraphMatrix(nodes);

  for(i = 0; i < nodes; i = i + 1){
    node = g.nodes[i];
    for(j = 0; j < node.edge.length; j = j + 1){
      edge = node.edge[j];
      m.c[i].r[edge.nodeNr] = edge.weight;
    }
  }

  return m;
}
function CreateDirectedGraphMatrix(nodes){
  var m;
  var i;
  m = {};

  m.c = [];
  m.c.length = nodes;
  for(i = 0; i < nodes; i = i + 1){
    m.c[i] = {};
    m.c[i].r = CreateNumberArray(nodes, 0);
  }

  return m;
}
function DirectedGraphsEqual(a, b){
  var equal, found, done;
  var nodes, foundCount, i, j, k, edges;
  var edgeA, edgeB;

  equal = true;

  if(a.nodes.length == b.nodes.length){
    nodes = a.nodes.length;

    done = false;
    for(i = 0; i < nodes &&  !done ; i = i + 1){
      if(GetEdgesForNodeFromDirectedGraph(a, i) == GetEdgesForNodeFromDirectedGraph(b, i)){
        edges = GetEdgesForNodeFromDirectedGraph(a, i);
        foundCount = 0;
        for(j = 0; j < edges &&  !done ; j = j + 1){
          found = false;
          for(k = 0; k < edges &&  !found ; k = k + 1){
            edgeA = GetEdgeFromDirectedGraph(a, i, j);
            edgeB = GetEdgeFromDirectedGraph(b, i, k);
            if(edgeA.nodeNr == edgeB.nodeNr){
              if(edgeA.weight == edgeB.weight){
                found = true;
              }
            }
          }
          if(found){
            foundCount = foundCount + 1;
          }else{
            equal = false;
            done = true;
          }
        }

        if(foundCount == edges){
        }else{
          equal = false;
          done = true;
        }
      }else{
        equal = false;
        done = true;
      }
    }
  }else{
    equal = false;
  }

  return equal;
}
function GetEdgesForNodeFromDirectedGraph(g, nodeNr){
  return g.nodes[nodeNr].edge.length;
}
function GetEdgeFromDirectedGraph(g, nodeNr, edgeNr){
  return g.nodes[nodeNr].edge[edgeNr];
}
function DirectedGraphMatricesEqual(a, b){
  var equal;
  var nodes, i, j;

  equal = true;

  if(GetNodesFromDirectedGraphMatrix(a) == GetNodesFromDirectedGraphMatrix(b)){
    nodes = GetNodesFromDirectedGraphMatrix(a);
    for(i = 0; i < nodes && equal; i = i + 1){
      for(j = 0; j < nodes && equal; j = j + 1){
        if(GetDirectedGraphMatrixEdge(a, i, j) == GetDirectedGraphMatrixEdge(b, i, j)){
        }else{
          equal = false;
        }
      }
    }
  }else{
    equal = false;
  }

  return equal;
}
function IsUndirected(g){
  var undirected, found;
  var u, i, v, j;
  var uNode, vNode;
  var uEdge, vEdge;

  undirected = true;

  for(u = 0; u < g.nodes.length; u = u + 1){
    uNode = g.nodes[u];
    for(i = 0; i < uNode.edge.length; i = i + 1){
      uEdge = uNode.edge[i];
      v = uEdge.nodeNr;

      if(u == v){
      }else{
        vNode = g.nodes[v];
        found = false;
        for(j = 0; j < vNode.edge.length &&  !found ; j = j + 1){
          vEdge = vNode.edge[j];

          if(vEdge.nodeNr == u && vEdge.weight == uEdge.weight){
            found = true;
          }
        }

        if( !found ){
          undirected = false;
        }
      }
    }
  }

  return undirected;
}
function GetGraphComponents(g, componentMembership){
  var valid, found;
  var i, nodeNr, componentNr, done, startNode;
  var componentMembershipSet;
  var list;

  valid = DirectedGraphIsValid(g) && IsUndirected(g);

  if(valid){
    componentMembership.numberArray = CreateNumberArray(g.nodes.length, 0);
    componentMembershipSet = CreateBooleanArray(g.nodes.length, false);
    list = {};
    componentNr = 0;
    done = 0;
    startNode = 0;

    for(; done != g.nodes.length; ){
      /* Find a node not currently in a component. */
      found = false;
      for(i = 0; i < g.nodes.length &&  !found ; i = i + 1){
        if( !componentMembershipSet[i] ){
          startNode = i;
          found = true;
        }
      }

      /* Use DFS to find a component */
      DepthFirstSearch(g, startNode, list);

      /* Record the component. */
      for(i = 0; i < list.numberArray.length; i = i + 1){
        nodeNr = list.numberArray[i];
        if( !componentMembershipSet[nodeNr] ){
          componentMembership.numberArray[nodeNr] = componentNr;
          componentMembershipSet[nodeNr] = true;
          done = done + 1;
        }
      }

      componentNr = componentNr + 1;
    }

    delete(componentMembershipSet);
  }

  return valid;
}
function TopologicalSort(g, list){
  var visited;
  var ll;
  var i;
  var valid;

  valid =  !DirectedGraphContainsCycleDFS(g) ;

  if(valid){

    visited = CreateBooleanArray(g.nodes.length, false);
    ll = CreateLinkedListNumbers();

    for(i = 0; i < g.nodes.length; i = i + 1){
      if( !visited[i] ){
        TopologicalSortRecursive(g, g.nodes[i], i, visited, ll);
      }
    }

    list.numberArray = LinkedListNumbersToArray(ll);
    FreeLinkedListNumbers(ll);

    for(i = 0; i < list.numberArray.length/2; i = i + 1){
      SwapElementsOfArray(list.numberArray, i, list.numberArray.length - i - 1);
    }
  }

  return valid;
}
function TopologicalSortRecursive(g, node, nodeNr, visited, list){
  var i;
  var e;

  visited[nodeNr] = true;

  for(i = 0; i < node.edge.length; i = i + 1){
    e = node.edge[i];
    if( !visited[e.nodeNr] ){
      TopologicalSortRecursive(g, g.nodes[e.nodeNr], e.nodeNr, visited, list);
    }
  }

  LinkedListAddNumber(list, nodeNr);
}
function ConvertPreviousListToForest(forest, prev){
  var length, i, j, root;
  var found;

  length = 0;
  for(i = 0; i < prev.length; i = i + 1){
    if(prev[i] == i){
      length = length + 1;
    }
  }

  forest.trees = [];
  forest.trees.length = length;

  j = 0;
  for(i = 0; i < length; i = i + 1){
    /* Find next root. */
    root = 0;
    found = false;
    for(; j < prev.length &&  !found ; j = j + 1){
      if(prev[j] == j){
        root = j;
        found = true;
      }
    }

    /* Create tree from root. */
    forest.trees[i] = ConvertPreviousListToTree(root, prev);
  }
}
function ConvertPreviousListToTree(root, prev){
  var tree;
  var branches, i, branch;

  tree = {};
  tree.label = root;

  /* Count branches. */
  branches = 0;
  for(i = 0; i < prev.length; i = i + 1){
    if(prev[i] == root && root != i){
      branches = branches + 1;
    }
  }

  /* Add branches. */
  tree.branches = [];
  tree.branches.length = branches;
  branch = 0;
  for(i = 0; i < prev.length; i = i + 1){
    if(prev[i] == root && root != i){
      tree.branches[branch] = ConvertPreviousListToTree(i, prev);
      branch = branch + 1;
    }
  }

  return tree;
}
function ConvertLinkedListTreesToForest(forest, roots, trees){
  var node;
  var length, current;

  node = roots.first;
  length = LinkedListNumbersLength(roots);
  forest.trees = [];
  forest.trees.length = length;

  for(current = 0;  !node.end ; current = current + 1){
    forest.trees[current] = ConvertLinkedListTreeToTree(node.value, trees);
    node = node.next;
  }
}
function ConvertLinkedListTreeToTree(root, trees){
  var tree;
  var rootList;
  var node;
  var current, length;

  rootList = trees[root];

  tree = {};
  tree.label = root;
  length = LinkedListNumbersLength(rootList);
  tree.branches = [];
  tree.branches.length = length;

  node = rootList.first;

  for(current = 0;  !node.end ; current = current + 1){
    tree.branches[current] = ConvertLinkedListTreeToTree(node.value, trees);
    node = node.next;
  }

  return tree;
}
function SpanningTreeAlgorithmsTest(failures){
  var g;
  var valid;
  var forest;

  forest = {};

  g = MakeUndirectedGraphForMST();
  valid = PrimsAlgorithmNoQueue(g, forest);
  CheckUndirectedGraphForMST(failures, valid, forest);
  valid = PrimsAlgorithm(g, forest);
  CheckUndirectedGraphForMST(failures, valid, forest);
  valid = KruskalsAlgorithm(g, forest);
  CheckUndirectedGraphForMSTKruskals(failures, valid, forest);

  g = MakeUndirectedGraph();
  valid = PrimsAlgorithm(g, forest);
  CheckUndirectedGraphMST(failures, valid, forest);
  valid = PrimsAlgorithmNoQueue(g, forest);
  CheckUndirectedGraphMST(failures, valid, forest);
  valid = KruskalsAlgorithm(g, forest);
  CheckUndirectedGraphMSTKruskals(failures, valid, forest);

  g = MakeUndirectedGraphWithThreeComponents();
  valid = PrimsAlgorithm(g, forest);
  CheckUndirectedGraphWithThreeComponentsMST(failures, valid, forest);
  valid = PrimsAlgorithmNoQueue(g, forest);
  CheckUndirectedGraphWithThreeComponentsMST(failures, valid, forest);
  valid = KruskalsAlgorithm(g, forest);
  CheckUndirectedGraphWithThreeComponentsMSTKruskals(failures, valid, forest);

  g = MakeUndirectedGraphForMST2();
  valid = PrimsAlgorithmNoQueue(g, forest);
  /*CheckUndirectedGraphForMST2(failures, valid, forest); */
  valid = PrimsAlgorithm(g, forest);
  CheckUndirectedGraphForMST2(failures, valid, forest);
  valid = KruskalsAlgorithm(g, forest);
  CheckUndirectedGraphForMST2Kruskals(failures, valid, forest);
}
function CheckUndirectedGraphWithThreeComponentsMST(failures, valid, forest){
  AssertTrue(valid, failures);
  AssertEquals(forest.trees.length, 3, failures);
  AssertEquals(forest.trees[0].label, 0, failures);
  AssertEquals(forest.trees[0].branches[0].label, 1, failures);
  AssertEquals(forest.trees[0].branches[0].branches[0].label, 2, failures);
  AssertEquals(forest.trees[1].label, 3, failures);
  AssertEquals(forest.trees[2].label, 4, failures);
  AssertEquals(forest.trees[2].branches[0].label, 5, failures);
  AssertEquals(forest.trees[2].branches[0].branches[0].label, 6, failures);
}
function CheckUndirectedGraphWithThreeComponentsMSTKruskals(failures, valid, forest){
  AssertTrue(valid, failures);
  AssertEquals(forest.trees.length, 3, failures);
  AssertEquals(forest.trees[0].label, 0, failures);
  AssertEquals(forest.trees[0].branches[0].label, 1, failures);
  AssertEquals(forest.trees[0].branches[0].branches[0].label, 2, failures);
  AssertEquals(forest.trees[1].label, 3, failures);
  AssertEquals(forest.trees[2].label, 6, failures);
  AssertEquals(forest.trees[2].branches[0].label, 5, failures);
  AssertEquals(forest.trees[2].branches[0].branches[0].label, 4, failures);
}
function CheckUndirectedGraphMST(failures, valid, forest){
  AssertTrue(valid, failures);
  AssertEquals(forest.trees.length, 1, failures);
  AssertEquals(forest.trees[0].label, 0, failures);
  AssertEquals(forest.trees[0].branches[0].label, 1, failures);
  AssertEquals(forest.trees[0].branches[0].branches[0].label, 2, failures);
}
function CheckUndirectedGraphMSTKruskals(failures, valid, forest){
  AssertTrue(valid, failures);
  AssertEquals(forest.trees.length, 1, failures);
  AssertEquals(forest.trees[0].label, 2, failures);
  AssertEquals(forest.trees[0].branches[0].label, 1, failures);
  AssertEquals(forest.trees[0].branches[0].branches[0].label, 0, failures);
}
function CheckUndirectedGraphForMST(failures, valid, forest){
  AssertTrue(valid, failures);
  AssertEquals(forest.trees.length, 2, failures);
  AssertEquals(forest.trees[1].label, 4, failures);
  AssertEquals(forest.trees[1].branches.length, 0, failures);
  AssertEquals(forest.trees[0].label, 0, failures);
  AssertEquals(forest.trees[0].branches.length, 2, failures);
  AssertEquals(forest.trees[0].branches[0].label, 3, failures);
  AssertEquals(forest.trees[0].branches[0].branches[0].label, 2, failures);
  AssertEquals(forest.trees[0].branches[1].label, 1, failures);
}
function CheckUndirectedGraphForMST2(failures, valid, forest){
  AssertTrue(valid, failures);
  AssertEquals(forest.trees.length, 2, failures);
  AssertEquals(forest.trees[1].label, 4, failures);
  AssertEquals(forest.trees[1].branches.length, 0, failures);
  AssertEquals(forest.trees[0].label, 0, failures);
  AssertEquals(forest.trees[0].branches[0].label, 3, failures);
  AssertEquals(forest.trees[0].branches[0].branches[0].label, 2, failures);
  AssertEquals(forest.trees[0].branches[1].label, 1, failures);
}
function CheckUndirectedGraphForMSTKruskals(failures, valid, forest){
  AssertTrue(valid, failures);
  AssertEquals(forest.trees.length, 2, failures);
  AssertEquals(forest.trees[1].label, 4, failures);
  AssertEquals(forest.trees[1].branches.length, 0, failures);
  AssertEquals(forest.trees[0].label, 0, failures);
  AssertEquals(forest.trees[0].branches.length, 1, failures);
  AssertEquals(forest.trees[0].branches[0].label, 3, failures);
  AssertEquals(forest.trees[0].branches[0].branches[0].label, 1, failures);
  AssertEquals(forest.trees[0].branches[0].branches[1].label, 2, failures);
}
function CheckUndirectedGraphForMST2Kruskals(failures, valid, forest){
  AssertTrue(valid, failures);
  AssertEquals(forest.trees.length, 2, failures);
  AssertEquals(forest.trees[1].label, 4, failures);
  AssertEquals(forest.trees[1].branches.length, 0, failures);
  AssertEquals(forest.trees[0].label, 2, failures);
  AssertEquals(forest.trees[0].branches[0].label, 3, failures);
  AssertEquals(forest.trees[0].branches[0].branches[0].label, 0, failures);
  AssertEquals(forest.trees[0].branches[0].branches[0].branches[0].label, 1, failures);
}
function searchTests(failures){
  DFSTest1(failures);
  DFSTest2(failures);
  DFSTest3(failures);

  BFSTest1(failures);
  BFSTest2(failures);
  BFSTest3(failures);
}
function DFSTest1(failures){
  var directedGraph;
  var list;

  directedGraph = MakeGraphWithTwoMixedCycles();

  list = {};

  DepthFirstSearch(directedGraph, 0, list);

  AssertEquals(list.numberArray[0], 0, failures);
  AssertEquals(list.numberArray[1], 1, failures);
  AssertEquals(list.numberArray[2], 2, failures);
  AssertEquals(list.numberArray[3], 3, failures);
}
function DFSTest2(failures){
  var directedGraph;
  var list;

  directedGraph = MakeGraphForDijkstrasAlgorithm();

  list = {};

  DepthFirstSearch(directedGraph, 0, list);

  AssertEquals(list.numberArray[0], 0, failures);
  AssertEquals(list.numberArray[1], 1, failures);
  AssertEquals(list.numberArray[2], 2, failures);
  AssertEquals(list.numberArray[3], 3, failures);
  AssertEquals(list.numberArray[4], 4, failures);
  AssertEquals(list.numberArray[5], 5, failures);
}
function DFSTest3(failures){
  var directedGraph;
  var list;

  directedGraph = MakeGraphForDijkstrasAlgorithm2();

  list = CreateNumberArrayReferenceLengthValue(directedGraph.nodes.length, 0);

  DepthFirstSearch(directedGraph, 0, list);

  AssertEquals(list.numberArray[0], 0, failures);
  AssertEquals(list.numberArray[1], 1, failures);
  AssertEquals(list.numberArray[2], 7, failures);
  AssertEquals(list.numberArray[3], 2, failures);
  AssertEquals(list.numberArray[4], 8, failures);
  AssertEquals(list.numberArray[5], 3, failures);
  AssertEquals(list.numberArray[6], 4, failures);
  AssertEquals(list.numberArray[7], 5, failures);
  AssertEquals(list.numberArray[8], 6, failures);
}
function BFSTest1(failures){
  var directedGraph;
  var list;

  directedGraph = MakeGraphWithTwoMixedCycles();

  list = {};

  BreadthFirstSearch(directedGraph, 0, list);

  AssertEquals(list.numberArray[0], 0, failures);
  AssertEquals(list.numberArray[1], 1, failures);
  AssertEquals(list.numberArray[2], 2, failures);
  AssertEquals(list.numberArray[3], 3, failures);
}
function BFSTest2(failures){
  var directedGraph;
  var list;

  directedGraph = MakeGraphForDijkstrasAlgorithm();

  list = {};

  BreadthFirstSearch(directedGraph, 0, list);

  AssertEquals(list.numberArray[0], 0, failures);
  AssertEquals(list.numberArray[1], 1, failures);
  AssertEquals(list.numberArray[2], 2, failures);
  AssertEquals(list.numberArray[3], 5, failures);
  AssertEquals(list.numberArray[4], 3, failures);
  AssertEquals(list.numberArray[5], 4, failures);
}
function BFSTest3(failures){
  var directedGraph;
  var list;

  directedGraph = MakeGraphForDijkstrasAlgorithm2();

  list = {};

  BreadthFirstSearch(directedGraph, 0, list);

  AssertEquals(list.numberArray[0], 0, failures);
  AssertEquals(list.numberArray[1], 1, failures);
  AssertEquals(list.numberArray[2], 7, failures);
  AssertEquals(list.numberArray[3], 6, failures);
  AssertEquals(list.numberArray[4], 2, failures);
  AssertEquals(list.numberArray[5], 8, failures);
  AssertEquals(list.numberArray[6], 5, failures);
  AssertEquals(list.numberArray[7], 3, failures);
  AssertEquals(list.numberArray[8], 4, failures);
}
function ShortestPathsTests(failures){
  testDijkstrasAlgorithm(failures);
  testDijkstrasAlgorithm2(failures);
  testBellmanFordAlgorithm(failures);
  testBellmanFordAlgorithm2(failures);
  testFloydWarshallAlgorithm(failures);
  testFloydWarshallAlgorithm2(failures);
  testShortestPathAlgorithmsDistanceOnly(failures);
}
function testDijkstrasAlgorithm(failures){
  var directedGraph;
  var d, p;
  var ds;

  directedGraph = MakeGraphForDijkstrasAlgorithm();

  d = {};
  p = {};
  ds = {};
  DijkstrasAlgorithm(directedGraph, 0, d, ds, p);

  CheckShortestPath(failures, d, p);
}
function testShortestPathAlgorithmsDistanceOnly(failures){
  var directedGraph;
  var path;
  var distance;
  var success;

  directedGraph = MakeGraphForDijkstrasAlgorithm();

  path = {};
  distance = {};
  success = DijkstrasAlgorithmDestinationOnly(directedGraph, 0, 5, path, distance);

  AssertTrue(success, failures);
  AssertEquals(distance.numberValue, 11, failures);
  AssertEquals(path.numberArray.length, 3, failures);
  AssertEquals(path.numberArray[0], 0, failures);
  AssertEquals(path.numberArray[1], 2, failures);
  AssertEquals(path.numberArray[2], 5, failures);

  path = {};
  distance = {};
  success = BellmanFordAlgorithmDestinationOnly(directedGraph, 0, 5, path, distance);

  AssertTrue(success, failures);
  AssertEquals(distance.numberValue, 11, failures);
  AssertEquals(path.numberArray.length, 3, failures);
  AssertEquals(path.numberArray[0], 0, failures);
  AssertEquals(path.numberArray[1], 2, failures);
  AssertEquals(path.numberArray[2], 5, failures);
}
function testBellmanFordAlgorithm(failures){
  var directedGraph;
  var d, p;
  var ds;
  var success;

  directedGraph = MakeGraphForDijkstrasAlgorithm();

  d = {};
  p = {};
  ds = {};
  success = BellmanFordAlgorithm(directedGraph, 0, d, ds, p);

  AssertTrue(success, failures);
  CheckShortestPath(failures, d, p);
}
function CheckShortestPath(failures, d, p){
  AssertEquals(d.numberArray[0], 0, failures);
  AssertEquals(d.numberArray[1], 7, failures);
  AssertEquals(d.numberArray[2], 9, failures);
  AssertEquals(d.numberArray[3], 20, failures);
  AssertEquals(d.numberArray[4], 20, failures);
  AssertEquals(d.numberArray[5], 11, failures);

  AssertEquals(p.numberArray[0], 0, failures);
  AssertEquals(p.numberArray[1], 0, failures);
  AssertEquals(p.numberArray[2], 0, failures);
  AssertEquals(p.numberArray[3], 2, failures);
  AssertEquals(p.numberArray[4], 5, failures);
  AssertEquals(p.numberArray[5], 2, failures);
}
function MakeGraphForDijkstrasAlgorithm(){
  var directedGraph;

  directedGraph = CreateDirectedGraph(6);

  directedGraph.nodes[0].edge = [];
  directedGraph.nodes[0].edge.length = 3;
  directedGraph.nodes[0].edge[0] = CreateEdge(1, 7);
  directedGraph.nodes[0].edge[1] = CreateEdge(2, 9);
  directedGraph.nodes[0].edge[2] = CreateEdge(5, 14);

  directedGraph.nodes[1].edge = [];
  directedGraph.nodes[1].edge.length = 3;
  directedGraph.nodes[1].edge[0] = CreateEdge(0, 7);
  directedGraph.nodes[1].edge[1] = CreateEdge(2, 10);
  directedGraph.nodes[1].edge[2] = CreateEdge(3, 15);

  directedGraph.nodes[2].edge = [];
  directedGraph.nodes[2].edge.length = 4;
  directedGraph.nodes[2].edge[0] = CreateEdge(0, 9);
  directedGraph.nodes[2].edge[1] = CreateEdge(1, 10);
  directedGraph.nodes[2].edge[2] = CreateEdge(3, 11);
  directedGraph.nodes[2].edge[3] = CreateEdge(5, 2);

  directedGraph.nodes[3].edge = [];
  directedGraph.nodes[3].edge.length = 3;
  directedGraph.nodes[3].edge[0] = CreateEdge(1, 15);
  directedGraph.nodes[3].edge[1] = CreateEdge(2, 11);
  directedGraph.nodes[3].edge[2] = CreateEdge(4, 6);

  directedGraph.nodes[4].edge = [];
  directedGraph.nodes[4].edge.length = 2;
  directedGraph.nodes[4].edge[0] = CreateEdge(3, 6);
  directedGraph.nodes[4].edge[1] = CreateEdge(5, 9);

  directedGraph.nodes[5].edge = [];
  directedGraph.nodes[5].edge.length = 3;
  directedGraph.nodes[5].edge[0] = CreateEdge(0, 14);
  directedGraph.nodes[5].edge[1] = CreateEdge(2, 2);
  directedGraph.nodes[5].edge[2] = CreateEdge(4, 9);

  return directedGraph;
}
function testDijkstrasAlgorithm2(failures){
  var directedGraph;
  var d, p;
  var ds;

  directedGraph = MakeGraphForDijkstrasAlgorithm2();
  d = {};
  p = {};
  ds = {};
  DijkstrasAlgorithm(directedGraph, 0, d, ds, p);

  CheckShortestPath2(failures, d, p);
}
function testBellmanFordAlgorithm2(failures){
  var directedGraph;
  var d, p;
  var ds;

  directedGraph = MakeGraphForDijkstrasAlgorithm2();
  d = {};
  p = {};
  ds = {};
  BellmanFordAlgorithm(directedGraph, 0, d, ds, p);

  CheckShortestPath2(failures, d, p);
}
function CheckShortestPath2(failures, d, p){
  AssertEquals(d.numberArray[0], 0, failures);
  AssertEquals(d.numberArray[1], 5, failures);
  AssertEquals(d.numberArray[2], 7, failures);
  AssertEquals(d.numberArray[3], 7, failures);
  AssertEquals(d.numberArray[4], 7, failures);
  AssertEquals(d.numberArray[5], 6, failures);
  AssertEquals(d.numberArray[6], 3, failures);
  AssertEquals(d.numberArray[7], 4, failures);
  AssertEquals(d.numberArray[8], 5, failures);

  AssertEquals(p.numberArray[0], 0, failures);
  AssertEquals(p.numberArray[1], 0, failures);
  AssertTrue(p.numberArray[2] == 7 || p.numberArray[2] == 1, failures);
  AssertEquals(p.numberArray[3], 8, failures);
  AssertEquals(p.numberArray[4], 5, failures);
  AssertEquals(p.numberArray[5], 7, failures);
  AssertEquals(p.numberArray[6], 0, failures);
  AssertEquals(p.numberArray[7], 6, failures);
  AssertEquals(p.numberArray[8], 7, failures);
}
function MakeGraphForDijkstrasAlgorithm2(){
  var directedGraph;

  directedGraph = CreateDirectedGraph(9);

  directedGraph.nodes[0].edge = [];
  directedGraph.nodes[0].edge.length = 3;
  directedGraph.nodes[0].edge[0] = CreateEdge(1, 5);
  directedGraph.nodes[0].edge[1] = CreateEdge(7, 7);
  directedGraph.nodes[0].edge[2] = CreateEdge(6, 3);

  directedGraph.nodes[1].edge = [];
  directedGraph.nodes[1].edge.length = 3;
  directedGraph.nodes[1].edge[0] = CreateEdge(0, 5);
  directedGraph.nodes[1].edge[1] = CreateEdge(7, 3);
  directedGraph.nodes[1].edge[2] = CreateEdge(2, 2);

  directedGraph.nodes[2].edge = [];
  directedGraph.nodes[2].edge.length = 4;
  directedGraph.nodes[2].edge[0] = CreateEdge(1, 2);
  directedGraph.nodes[2].edge[1] = CreateEdge(7, 3);
  directedGraph.nodes[2].edge[2] = CreateEdge(8, 3);
  directedGraph.nodes[2].edge[3] = CreateEdge(3, 4);

  directedGraph.nodes[3].edge = [];
  directedGraph.nodes[3].edge.length = 3;
  directedGraph.nodes[3].edge[0] = CreateEdge(2, 4);
  directedGraph.nodes[3].edge[1] = CreateEdge(8, 2);
  directedGraph.nodes[3].edge[2] = CreateEdge(4, 5);

  directedGraph.nodes[4].edge = [];
  directedGraph.nodes[4].edge.length = 3;
  directedGraph.nodes[4].edge[0] = CreateEdge(3, 5);
  directedGraph.nodes[4].edge[1] = CreateEdge(8, 3);
  directedGraph.nodes[4].edge[2] = CreateEdge(5, 1);

  directedGraph.nodes[5].edge = [];
  directedGraph.nodes[5].edge.length = 4;
  directedGraph.nodes[5].edge[0] = CreateEdge(4, 1);
  directedGraph.nodes[5].edge[1] = CreateEdge(8, 2);
  directedGraph.nodes[5].edge[2] = CreateEdge(7, 2);
  directedGraph.nodes[5].edge[3] = CreateEdge(6, 7);

  directedGraph.nodes[6].edge = [];
  directedGraph.nodes[6].edge.length = 3;
  directedGraph.nodes[6].edge[0] = CreateEdge(0, 3);
  directedGraph.nodes[6].edge[1] = CreateEdge(7, 1);
  directedGraph.nodes[6].edge[2] = CreateEdge(5, 7);

  directedGraph.nodes[7].edge = [];
  directedGraph.nodes[7].edge.length = 6;
  directedGraph.nodes[7].edge[0] = CreateEdge(0, 7);
  directedGraph.nodes[7].edge[1] = CreateEdge(1, 3);
  directedGraph.nodes[7].edge[2] = CreateEdge(2, 3);
  directedGraph.nodes[7].edge[3] = CreateEdge(8, 1);
  directedGraph.nodes[7].edge[4] = CreateEdge(5, 2);
  directedGraph.nodes[7].edge[5] = CreateEdge(6, 1);

  directedGraph.nodes[8].edge = [];
  directedGraph.nodes[8].edge.length = 5;
  directedGraph.nodes[8].edge[0] = CreateEdge(3, 2);
  directedGraph.nodes[8].edge[1] = CreateEdge(2, 3);
  directedGraph.nodes[8].edge[2] = CreateEdge(7, 1);
  directedGraph.nodes[8].edge[3] = CreateEdge(5, 2);
  directedGraph.nodes[8].edge[4] = CreateEdge(4, 3);

  return directedGraph;
}
function testFloydWarshallAlgorithm(failures){
  var directedGraph;
  var distances;
  var success;
  var d, p;
  var i;
  var path;

  directedGraph = MakeGraphForDijkstrasAlgorithm();

  distances = CreateDistancesFloydWarshallAlgorithm(directedGraph.nodes.length);
  success = FloydWarshallAlgorithm(directedGraph, distances);

  AssertTrue(success, failures);

  d = CreateNumberArrayReferenceLengthValue(directedGraph.nodes.length, 0);
  p = CreateNumberArrayReferenceLengthValue(directedGraph.nodes.length, 0);

  for(i = 0; i < directedGraph.nodes.length; i = i + 1){
    d.numberArray[i] = distances.from[0].to[i].length;
    path = GetPathFromDistances(distances, 0, i);
    if(path.length >= 2){
      p.numberArray[i] = path[path.length - 2];
    }else{
      p.numberArray[i] = i;
    }
  }

  CheckShortestPath(failures, d, p);
}
function testFloydWarshallAlgorithm2(failures){
  var directedGraph;
  var distances;
  var success;
  var d, p;
  var i;
  var path;

  directedGraph = MakeGraphForDijkstrasAlgorithm2();

  distances = CreateDistancesFloydWarshallAlgorithm(directedGraph.nodes.length);
  success = FloydWarshallAlgorithm(directedGraph, distances);

  AssertTrue(success, failures);

  d = CreateNumberArrayReferenceLengthValue(directedGraph.nodes.length, 0);
  p = CreateNumberArrayReferenceLengthValue(directedGraph.nodes.length, 0);

  for(i = 0; i < directedGraph.nodes.length; i = i + 1){
    d.numberArray[i] = distances.from[0].to[i].length;
    path = GetPathFromDistances(distances, 0, i);
    if(path.length >= 2){
      p.numberArray[i] = path[path.length - 2];
    }else{
      p.numberArray[i] = i;
    }
  }

  CheckShortestPath2(failures, d, p);
}
function test(){
  var failures;

  failures = CreateNumberReference(0);

  testOneCycle(failures);
  testNoCycle(failures);
  testTwoCycles(failures);
  testOneSelfCycle(failures);
  testTwoMixedCycles(failures);
  testMatrixFormConversions(failures);
  ShortestPathsTests(failures);
  searchTests(failures);
  IsUndirectedTests(failures);
  GraphComponentsTest(failures);
  SpanningTreeAlgorithmsTest(failures);
  TestTopologicalSort(failures);

  return failures.numberValue;
}
function testOneSelfCycle(failures){
  var directedGraph;
  var valid, cycle;
  var cycleCount;
  var cycles;

  directedGraph = MakeGraphWithOneSelfCycle();
  valid = DirectedGraphIsValid(directedGraph);
  AssertTrue(valid, failures);
  cycle = DirectedGraphContainsCycleDFS(directedGraph);
  AssertTrue(cycle, failures);
  cycleCount = DirectedGraphCountCyclesDFS(directedGraph);
  AssertEquals(1, cycleCount, failures);
  cycles = DirectedGraphGetCyclesDFS(directedGraph);
  AssertEquals(1, cycles.length, failures);
  AssertEquals(1, cycles[0].nodeNrs.length, failures);
  AssertEquals(2, cycles[0].nodeNrs[0], failures);
}
function testTwoCycles(failures){
  var directedGraph;
  var valid, cycle;
  var cycleCount;
  var cycles;

  directedGraph = MakeGraphWithTwoCycles();
  valid = DirectedGraphIsValid(directedGraph);
  AssertTrue(valid, failures);
  cycle = DirectedGraphContainsCycleDFS(directedGraph);
  AssertTrue(cycle, failures);
  cycleCount = DirectedGraphCountCyclesDFS(directedGraph);
  AssertEquals(2, cycleCount, failures);
  cycles = DirectedGraphGetCyclesDFS(directedGraph);
  AssertEquals(2, cycles.length, failures);
  AssertEquals(3, cycles[0].nodeNrs.length, failures);
  AssertEquals(1, cycles[0].nodeNrs[0], failures);
  AssertEquals(2, cycles[0].nodeNrs[1], failures);
  AssertEquals(3, cycles[0].nodeNrs[2], failures);
  AssertEquals(2, cycles[1].nodeNrs.length, failures);
  AssertEquals(0, cycles[1].nodeNrs[0], failures);
  AssertEquals(4, cycles[1].nodeNrs[1], failures);
}
function testNoCycle(failures){
  var directedGraph;
  var valid, cycle;
  var cycleCount;
  var cycles;

  directedGraph = MakeGraphWithoutCycle();
  valid = DirectedGraphIsValid(directedGraph);
  AssertTrue(valid, failures);
  cycle = DirectedGraphContainsCycleDFS(directedGraph);
  AssertFalse(cycle, failures);
  cycleCount = DirectedGraphCountCyclesDFS(directedGraph);
  AssertEquals(0, cycleCount, failures);
  cycles = DirectedGraphGetCyclesDFS(directedGraph);
  AssertEquals(0, cycles.length, failures);
}
function testOneCycle(failures){
  var directedGraph;
  var valid, cycle;
  var cycleCount;
  var cycles;

  directedGraph = MakeGraphWithOneCycle();
  valid = DirectedGraphIsValid(directedGraph);
  AssertTrue(valid, failures);
  cycle = DirectedGraphContainsCycleDFS(directedGraph);
  AssertTrue(cycle, failures);
  cycleCount = DirectedGraphCountCyclesDFS(directedGraph);
  AssertEquals(1, cycleCount, failures);
  cycles = DirectedGraphGetCyclesDFS(directedGraph);
  AssertEquals(1, cycles.length, failures);
  AssertEquals(3, cycles[0].nodeNrs.length, failures);
  AssertEquals(1, cycles[0].nodeNrs[0], failures);
  AssertEquals(2, cycles[0].nodeNrs[1], failures);
  AssertEquals(3, cycles[0].nodeNrs[2], failures);
}
function testTwoMixedCycles(failures){
  var directedGraph;
  var valid, cycle;
  var cycleCount;
  var cycles;

  directedGraph = MakeGraphWithTwoMixedCycles();
  valid = DirectedGraphIsValid(directedGraph);
  AssertTrue(valid, failures);
  cycle = DirectedGraphContainsCycleDFS(directedGraph);
  AssertTrue(cycle, failures);
  cycleCount = DirectedGraphCountCyclesDFS(directedGraph);
  AssertEquals(2, cycleCount, failures);
  cycles = DirectedGraphGetCyclesDFS(directedGraph);
  AssertEquals(2, cycles.length, failures);
  AssertEquals(3, cycles[0].nodeNrs.length, failures);
  AssertEquals(1, cycles[0].nodeNrs[0], failures);
  AssertEquals(2, cycles[0].nodeNrs[1], failures);
  AssertEquals(3, cycles[0].nodeNrs[2], failures);
  AssertEquals(1, cycles[1].nodeNrs.length, failures);
  AssertEquals(3, cycles[1].nodeNrs[0], failures);
}
function testMatrixFormConversions(failures){
  var a, b;
  var m1, m2;

  a = MakeGraphWithTwoMixedCycles();
  b = CreateDirectedGraphFromMatrixForm(CreateDirectedGraphMatrixFromListForm(MakeGraphWithTwoMixedCycles()));

  AssertTrue(DirectedGraphsEqual(a, b), failures);

  m1 = CreateDirectedGraphMatrixFromListForm(a);
  m2 = CreateDirectedGraphMatrixFromListForm(a);

  AssertTrue(DirectedGraphMatricesEqual(m1, m2), failures);
}
function IsUndirectedTests(failures){
  var g;
  var undirected;

  g = MakeGraphWithTwoMixedCycles();

  undirected = IsUndirected(g);
  AssertFalse(undirected, failures);

  g = MakeGraphWithOneSelfCycle();

  undirected = IsUndirected(g);
  AssertFalse(undirected, failures);

  g = MakeGraphWithOneCycle();

  undirected = IsUndirected(g);
  AssertFalse(undirected, failures);

  g = MakeGraphWithoutCycle();

  undirected = IsUndirected(g);
  AssertFalse(undirected, failures);

  g = MakeGraphWithTwoCycles();

  undirected = IsUndirected(g);
  AssertFalse(undirected, failures);

  g = MakeGraphWithTwoMixedCycles();

  undirected = IsUndirected(g);
  AssertFalse(undirected, failures);

  g = MakeUndirectedGraph();

  undirected = IsUndirected(g);
  AssertTrue(undirected, failures);

  g = MakeUndirectedGraphForMST();

  undirected = IsUndirected(g);
  AssertTrue(undirected, failures);
}
function GraphComponentsTest(failures){
  var g;
  var valid;
  var components;

  g = MakeUndirectedGraph();

  components = {};
  valid = GetGraphComponents(g, components);
  AssertTrue(valid, failures);

  AssertEquals(components.numberArray[0], 0, failures);
  AssertEquals(components.numberArray[1], 0, failures);
  AssertEquals(components.numberArray[2], 0, failures);

  g = MakeUndirectedGraphWithThreeComponents();

  components = {};
  valid = GetGraphComponents(g, components);
  AssertTrue(valid, failures);

  AssertEquals(components.numberArray[0], 0, failures);
  AssertEquals(components.numberArray[1], 0, failures);
  AssertEquals(components.numberArray[2], 0, failures);
  AssertEquals(components.numberArray[3], 1, failures);
  AssertEquals(components.numberArray[4], 2, failures);
  AssertEquals(components.numberArray[5], 2, failures);
  AssertEquals(components.numberArray[6], 2, failures);

  FreeNumberArrayReference(components);
}
function TestTopologicalSort(failures){
  var g;
  var valid;
  var list;

  g = MakeTopologicalSortGraph();
  list = {};

  valid = TopologicalSort(g, list);

  AssertTrue(valid, failures);
  AssertEquals(list.numberArray[0], 5, failures);
  AssertEquals(list.numberArray[1], 4, failures);
  AssertEquals(list.numberArray[2], 2, failures);
  AssertEquals(list.numberArray[3], 3, failures);
  AssertEquals(list.numberArray[4], 1, failures);
  AssertEquals(list.numberArray[5], 0, failures);
}
function MakeGraphWithOneSelfCycle(){
  var directedGraph;
  directedGraph = CreateDirectedGraph(4);

  directedGraph.nodes[0].edge = [];
  directedGraph.nodes[0].edge.length = 1;
  directedGraph.nodes[0].edge[0] = CreateEdge(1, 1);

  directedGraph.nodes[1].edge = [];
  directedGraph.nodes[1].edge.length = 2;
  directedGraph.nodes[1].edge[0] = CreateEdge(2, 1);
  directedGraph.nodes[1].edge[1] = CreateEdge(3, 1);

  directedGraph.nodes[2].edge = [];
  directedGraph.nodes[2].edge.length = 1;
  directedGraph.nodes[2].edge[0] = CreateEdge(2, 1);

  directedGraph.nodes[3].edge = [];
  directedGraph.nodes[3].edge.length = 0;

  return directedGraph;
}
function MakeGraphWithOneCycle(){
  var directedGraph;
  directedGraph = CreateDirectedGraph(4);

  directedGraph.nodes[0].edge = [];
  directedGraph.nodes[0].edge.length = 1;
  directedGraph.nodes[0].edge[0] = CreateEdge(1, 1);

  directedGraph.nodes[1].edge = [];
  directedGraph.nodes[1].edge.length = 1;
  directedGraph.nodes[1].edge[0] = CreateEdge(2, 1);

  directedGraph.nodes[2].edge = [];
  directedGraph.nodes[2].edge.length = 1;
  directedGraph.nodes[2].edge[0] = CreateEdge(3, 1);

  directedGraph.nodes[3].edge = [];
  directedGraph.nodes[3].edge.length = 1;
  directedGraph.nodes[3].edge[0] = CreateEdge(1, 1);

  return directedGraph;
}
function MakeGraphWithoutCycle(){
  var directedGraph;
  directedGraph = CreateDirectedGraph(3);

  directedGraph.nodes[0].edge = [];
  directedGraph.nodes[0].edge.length = 1;
  directedGraph.nodes[0].edge[0] = CreateEdge(1, 1);

  directedGraph.nodes[1].edge = [];
  directedGraph.nodes[1].edge.length = 1;
  directedGraph.nodes[1].edge[0] = CreateEdge(2, 1);

  directedGraph.nodes[2].edge = [];
  directedGraph.nodes[2].edge.length = 0;

  return directedGraph;
}
function MakeGraphWithTwoCycles(){
  var directedGraph;
  directedGraph = CreateDirectedGraph(5);

  directedGraph.nodes[0].edge = [];
  directedGraph.nodes[0].edge.length = 2;
  directedGraph.nodes[0].edge[0] = CreateEdge(1, 1);
  directedGraph.nodes[0].edge[1] = CreateEdge(4, 1);

  directedGraph.nodes[1].edge = [];
  directedGraph.nodes[1].edge.length = 1;
  directedGraph.nodes[1].edge[0] = CreateEdge(2, 1);

  directedGraph.nodes[2].edge = [];
  directedGraph.nodes[2].edge.length = 1;
  directedGraph.nodes[2].edge[0] = CreateEdge(3, 1);

  directedGraph.nodes[3].edge = [];
  directedGraph.nodes[3].edge.length = 1;
  directedGraph.nodes[3].edge[0] = CreateEdge(1, 1);

  directedGraph.nodes[4].edge = [];
  directedGraph.nodes[4].edge.length = 1;
  directedGraph.nodes[4].edge[0] = CreateEdge(0, 1);

  return directedGraph;
}
function MakeGraphWithTwoMixedCycles(){
  var directedGraph;
  directedGraph = CreateDirectedGraph(4);

  directedGraph.nodes[0].edge = [];
  directedGraph.nodes[0].edge.length = 1;
  directedGraph.nodes[0].edge[0] = CreateEdge(1, 1);

  directedGraph.nodes[1].edge = [];
  directedGraph.nodes[1].edge.length = 1;
  directedGraph.nodes[1].edge[0] = CreateEdge(2, 1);

  directedGraph.nodes[2].edge = [];
  directedGraph.nodes[2].edge.length = 1;
  directedGraph.nodes[2].edge[0] = CreateEdge(3, 1);

  directedGraph.nodes[3].edge = [];
  directedGraph.nodes[3].edge.length = 2;
  directedGraph.nodes[3].edge[0] = CreateEdge(1, 1);
  directedGraph.nodes[3].edge[1] = CreateEdge(3, 1);

  return directedGraph;
}
function MakeUndirectedGraph(){
  var directedGraph;
  directedGraph = CreateDirectedGraph(3);

  directedGraph.nodes[0].edge = [];
  directedGraph.nodes[0].edge.length = 2;
  directedGraph.nodes[0].edge[0] = CreateEdge(1, 1);
  directedGraph.nodes[0].edge[1] = CreateEdge(0, 1);

  directedGraph.nodes[1].edge = [];
  directedGraph.nodes[1].edge.length = 2;
  directedGraph.nodes[1].edge[0] = CreateEdge(0, 1);
  directedGraph.nodes[1].edge[1] = CreateEdge(2, 1);

  directedGraph.nodes[2].edge = [];
  directedGraph.nodes[2].edge.length = 1;
  directedGraph.nodes[2].edge[0] = CreateEdge(1, 1);

  return directedGraph;
}
function MakeUndirectedGraphWithThreeComponents(){
  var directedGraph;
  directedGraph = CreateDirectedGraph(7);

  directedGraph.nodes[0].edge = [];
  directedGraph.nodes[0].edge.length = 2;
  directedGraph.nodes[0].edge[0] = CreateEdge(1, 2);
  directedGraph.nodes[0].edge[1] = CreateEdge(0, 1);

  directedGraph.nodes[1].edge = [];
  directedGraph.nodes[1].edge.length = 2;
  directedGraph.nodes[1].edge[0] = CreateEdge(0, 2);
  directedGraph.nodes[1].edge[1] = CreateEdge(2, 1);

  directedGraph.nodes[2].edge = [];
  directedGraph.nodes[2].edge.length = 1;
  directedGraph.nodes[2].edge[0] = CreateEdge(1, 1);

  directedGraph.nodes[3].edge = [];
  directedGraph.nodes[3].edge.length = 0;

  directedGraph.nodes[4].edge = [];
  directedGraph.nodes[4].edge.length = 2;
  directedGraph.nodes[4].edge[0] = CreateEdge(5, 1);
  directedGraph.nodes[4].edge[1] = CreateEdge(4, 1);

  directedGraph.nodes[5].edge = [];
  directedGraph.nodes[5].edge.length = 2;
  directedGraph.nodes[5].edge[0] = CreateEdge(4, 1);
  directedGraph.nodes[5].edge[1] = CreateEdge(6, 1);

  directedGraph.nodes[6].edge = [];
  directedGraph.nodes[6].edge.length = 1;
  directedGraph.nodes[6].edge[0] = CreateEdge(5, 1);

  return directedGraph;
}
function MakeUndirectedGraphForMST(){
  var directedGraph;
  directedGraph = CreateDirectedGraph(5);

  directedGraph.nodes[0].edge = [];
  directedGraph.nodes[0].edge.length = 2;
  directedGraph.nodes[0].edge[0] = CreateEdge(1, 2);
  directedGraph.nodes[0].edge[1] = CreateEdge(3, 1);

  directedGraph.nodes[1].edge = [];
  directedGraph.nodes[1].edge.length = 2;
  directedGraph.nodes[1].edge[0] = CreateEdge(0, 2);
  directedGraph.nodes[1].edge[1] = CreateEdge(3, 2);

  directedGraph.nodes[2].edge = [];
  directedGraph.nodes[2].edge.length = 1;
  directedGraph.nodes[2].edge[0] = CreateEdge(3, 3);

  directedGraph.nodes[3].edge = [];
  directedGraph.nodes[3].edge.length = 3;
  directedGraph.nodes[3].edge[0] = CreateEdge(0, 1);
  directedGraph.nodes[3].edge[1] = CreateEdge(1, 2);
  directedGraph.nodes[3].edge[2] = CreateEdge(2, 3);

  directedGraph.nodes[4].edge = [];
  directedGraph.nodes[4].edge.length = 0;

  return directedGraph;
}
function MakeUndirectedGraphForMST2(){
  var directedGraph;
  directedGraph = CreateDirectedGraph(5);

  directedGraph.nodes[0].edge = [];
  directedGraph.nodes[0].edge.length = 2;
  directedGraph.nodes[0].edge[0] = CreateEdge(1, 2);
  directedGraph.nodes[0].edge[1] = CreateEdge(3, 1);

  directedGraph.nodes[1].edge = [];
  directedGraph.nodes[1].edge.length = 2;
  directedGraph.nodes[1].edge[0] = CreateEdge(0, 2);
  directedGraph.nodes[1].edge[1] = CreateEdge(3, 4);

  directedGraph.nodes[2].edge = [];
  directedGraph.nodes[2].edge.length = 1;
  directedGraph.nodes[2].edge[0] = CreateEdge(3, 3);

  directedGraph.nodes[3].edge = [];
  directedGraph.nodes[3].edge.length = 3;
  directedGraph.nodes[3].edge[0] = CreateEdge(0, 1);
  directedGraph.nodes[3].edge[1] = CreateEdge(1, 4);
  directedGraph.nodes[3].edge[2] = CreateEdge(2, 3);

  directedGraph.nodes[4].edge = [];
  directedGraph.nodes[4].edge.length = 0;

  return directedGraph;
}
function MakeTopologicalSortGraph(){
  var directedGraph;
  directedGraph = CreateDirectedGraph(6);

  directedGraph.nodes[0].edge = [];
  directedGraph.nodes[0].edge.length = 0;

  directedGraph.nodes[1].edge = [];
  directedGraph.nodes[1].edge.length = 0;

  directedGraph.nodes[2].edge = [];
  directedGraph.nodes[2].edge.length = 1;
  directedGraph.nodes[2].edge[0] = CreateEdge(3, 1);

  directedGraph.nodes[3].edge = [];
  directedGraph.nodes[3].edge.length = 1;
  directedGraph.nodes[3].edge[0] = CreateEdge(1, 1);

  directedGraph.nodes[4].edge = [];
  directedGraph.nodes[4].edge.length = 2;
  directedGraph.nodes[4].edge[0] = CreateEdge(0, 1);
  directedGraph.nodes[4].edge[1] = CreateEdge(1, 1);

  directedGraph.nodes[5].edge = [];
  directedGraph.nodes[5].edge.length = 2;
  directedGraph.nodes[5].edge[0] = CreateEdge(0, 1);
  directedGraph.nodes[5].edge[1] = CreateEdge(2, 1);

  return directedGraph;
}
function StringToNumberArray(string){
  var i;
  var array;

  array = [];
  array.length = string.length;

  for(i = 0; i < string.length; i = i + 1){
    array[i] = string[i].charCodeAt(0);
  }
  return array;
}
function NumberArrayToString(array){
  var i;
  var string;

  string = [];
  string.length = array.length;

  for(i = 0; i < array.length; i = i + 1){
    string[i] = String.fromCharCode(array[i]);
  }
  return string;
}
function NumberArraysEqual(a, b){
  var equal;
  var i;

  equal = true;
  if(a.length == b.length){
    for(i = 0; i < a.length && equal; i = i + 1){
      if(a[i] != b[i]){
        equal = false;
      }
    }
  }else{
    equal = false;
  }

  return equal;
}
function BooleanArraysEqual(a, b){
  var equal;
  var i;

  equal = true;
  if(a.length == b.length){
    for(i = 0; i < a.length && equal; i = i + 1){
      if(a[i] != b[i]){
        equal = false;
      }
    }
  }else{
    equal = false;
  }

  return equal;
}
function StringsEqual(a, b){
  var equal;
  var i;

  equal = true;
  if(a.length == b.length){
    for(i = 0; i < a.length && equal; i = i + 1){
      if(a[i] != b[i]){
        equal = false;
      }
    }
  }else{
    equal = false;
  }

  return equal;
}
function FillNumberArray(a, value){
  var i;

  for(i = 0; i < a.length; i = i + 1){
    a[i] = value;
  }
}
function FillString(a, value){
  var i;

  for(i = 0; i < a.length; i = i + 1){
    a[i] = value;
  }
}
function FillBooleanArray(a, value){
  var i;

  for(i = 0; i < a.length; i = i + 1){
    a[i] = value;
  }
}
function FillNumberArrayRange(a, value, from, to){
  var i, length;
  var success;

  if(from >= 0 && from <= a.length && to >= 0 && to <= a.length && from <= to){
    length = to - from;
    for(i = 0; i < length; i = i + 1){
      a[from + i] = value;
    }

    success = true;
  }else{
    success = false;
  }

  return success;
}
function FillBooleanArrayRange(a, value, from, to){
  var i, length;
  var success;

  if(from >= 0 && from <= a.length && to >= 0 && to <= a.length && from <= to){
    length = to - from;
    for(i = 0; i < length; i = i + 1){
      a[from + i] = value;
    }

    success = true;
  }else{
    success = false;
  }

  return success;
}
function FillStringRange(a, value, from, to){
  var i, length;
  var success;

  if(from >= 0 && from <= a.length && to >= 0 && to <= a.length && from <= to){
    length = to - from;
    for(i = 0; i < length; i = i + 1){
      a[from + i] = value;
    }

    success = true;
  }else{
    success = false;
  }

  return success;
}
function CopyNumberArray(a){
  var i;
  var n;

  n = [];
  n.length = a.length;

  for(i = 0; i < a.length; i = i + 1){
    n[i] = a[i];
  }

  return n;
}
function CopyBooleanArray(a){
  var i;
  var n;

  n = [];
  n.length = a.length;

  for(i = 0; i < a.length; i = i + 1){
    n[i] = a[i];
  }

  return n;
}
function CopyString(a){
  var i;
  var n;

  n = [];
  n.length = a.length;

  for(i = 0; i < a.length; i = i + 1){
    n[i] = a[i];
  }

  return n;
}
function CopyNumberArrayRange(a, from, to, copyReference){
  var i, length;
  var n;
  var success;

  if(from >= 0 && from <= a.length && to >= 0 && to <= a.length && from <= to){
    length = to - from;
    n = [];
    n.length = length;

    for(i = 0; i < length; i = i + 1){
      n[i] = a[from + i];
    }

    copyReference.numberArray = n;
    success = true;
  }else{
    success = false;
  }

  return success;
}
function CopyBooleanArrayRange(a, from, to, copyReference){
  var i, length;
  var n;
  var success;

  if(from >= 0 && from <= a.length && to >= 0 && to <= a.length && from <= to){
    length = to - from;
    n = [];
    n.length = length;

    for(i = 0; i < length; i = i + 1){
      n[i] = a[from + i];
    }

    copyReference.booleanArray = n;
    success = true;
  }else{
    success = false;
  }

  return success;
}
function CopyStringRange(a, from, to, copyReference){
  var i, length;
  var n;
  var success;

  if(from >= 0 && from <= a.length && to >= 0 && to <= a.length && from <= to){
    length = to - from;
    n = [];
    n.length = length;

    for(i = 0; i < length; i = i + 1){
      n[i] = a[from + i];
    }

    copyReference.string = n;
    success = true;
  }else{
    success = false;
  }

  return success;
}
function IsLastElement(length, index){
  return index + 1 == length;
}
function CreateNumberArray(length, value){
  var array;

  array = [];
  array.length = length;
  FillNumberArray(array, value);

  return array;
}
function CreateBooleanArray(length, value){
  var array;

  array = [];
  array.length = length;
  FillBooleanArray(array, value);

  return array;
}
function CreateString(length, value){
  var array;

  array = [];
  array.length = length;
  FillString(array, value);

  return array;
}
function SwapElementsOfArray(A, ai, bi){
  var tmp;

  tmp = A[ai];
  A[ai] = A[bi];
  A[bi] = tmp;
}
function Negate(x){
  return  -x;
}
function Positive(x){
  return  +x;
}
function Factorial(x){
  var i, f;
  f = 1;

  for(i = 2; i <= x; i = i + 1){
    f = f*i;
  }

  return f;
}
function Round(x){
  return Math.floor(x + 0.5);
}
function BankersRound(x){
  var r;
  if(Absolute(x - Truncate(x)) == 0.5){
    if( !DivisibleBy(Round(x), 2) ){
      r = Round(x) - 1;
    }else{
      r = Round(x);
    }
  }else{
    r = Round(x);
  }

  return r;
}
function Ceil(x){
  return Math.ceil(x);
}
function Floor(x){
  return Math.floor(x);
}
function Truncate(x){
  var t;
  if(x >= 0){
    t = Math.floor(x);
  }else{
    t = Math.ceil(x);
  }

  return t;
}
function Absolute(x){
  return Math.abs(x);
}
function Logarithm(x){
  return Math.log10(x);
}
function NaturalLogarithm(x){
  return Math.log(x);
}
function Sin(x){
  return Math.sin(x);
}
function Cos(x){
  return Math.cos(x);
}
function Tan(x){
  return Math.tan(x);
}
function Asin(x){
  return Math.asin(x);
}
function Acos(x){
  return Math.acos(x);
}
function Atan(x){
  return Math.atan(x);
}
function Atan2(y, x){
  var a;
  a = 0;

  if(x > 0){
    a = Atan(y/x);
  }else if(x < 0 && y >= 0){
    a = Atan(y/x) + Math.PI;
  }else if(x < 0 && y < 0){
    a = Atan(y/x) - Math.PI;
  }else if(x == 0 && y > 0){
    a = Math.PI/2;
  }else if(x == 0 && y < 0){
    a =  -Math.PI/2;
  }

  return a;
}
function Squareroot(x){
  return Math.sqrt(x);
}
function Exp(x){
  return Math.exp(x);
}
function DivisibleBy(a, b){
  return ((a%b) == 0);
}
function Combinations(n, k){
  return Factorial(n)/(Factorial(n - k)*Factorial(k));
}
function EpsilonCompareApproximateDigits(a, b, digits){
  var ad, bd, d, epsilon;
  var ret;
  if(a < 0 && b < 0 || a > 0 && b > 0){
    if(a < 0 && b < 0){
      a =  -a;
      b =  -b;
    }
    ad = Math.log10(a);
    bd = Math.log10(b);
    d = Math.max(ad, bd);
    epsilon = Math.pow(10, d - digits);
    ret = Math.abs(a - b) > epsilon;
  }else{
    ret = false;
  }

  return ret;
}
function EpsilonCompare(a, b, epsilon){
  return Math.abs(a - b) < epsilon;
}
function GreatestCommonDivisor(a, b){
  var t;
  for(; b != 0; ){
    t = b;
    b = a%b;
    a = t;
  }

  return a;
}
function IsInteger(a){
  return (a - Math.floor(a)) == 0;
}
function GreatestCommonDivisorWithCheck(a, b, gcdReference){
  var success;
  var gcd;
  if(IsInteger(a) && IsInteger(b)){
    gcd = GreatestCommonDivisor(a, b);
    gcdReference.numberValue = gcd;
    success = true;
  }else{
    success = false;
  }

  return success;
}
function LeastCommonMultiple(a, b){
  var lcm;
  if(a > 0 && b > 0){
    lcm = Math.abs(a*b)/GreatestCommonDivisor(a, b);
  }else{
    lcm = 0;
  }

  return lcm;
}
function Sign(a){
  var s;
  if(a > 0){
    s = 1;
  }else if(a < 0){
    s =  -1;
  }else{
    s = 0;
  }

  return s;
}
function Max(a, b){
  return Math.max(a, b);
}
function Min(a, b){
  return Math.min(a, b);
}
function Power(a, b){
  return Math.pow(a, b);
}
function CreateBooleanReference(value){
  var ref;
  ref = {};
  ref.booleanValue = value;

  return ref;
}
function CreateBooleanArrayReference(value){
  var ref;
  ref = {};
  ref.booleanArray = value;

  return ref;
}
function CreateBooleanArrayReferenceLengthValue(length, value){
  var ref;
  var i;
  ref = {};
  ref.booleanArray = [];
  ref.booleanArray.length = length;

  for(i = 0; i < length; i = i + 1){
    ref.booleanArray[i] = value;
  }

  return ref;
}
function FreeBooleanArrayReference(booleanArrayReference){
  delete(booleanArrayReference.booleanArray);
  delete(booleanArrayReference);
}
function CreateCharacterReference(value){
  var ref;
  ref = {};
  ref.characterValue = value;

  return ref;
}
function CreateNumberReference(value){
  var ref;
  ref = {};
  ref.numberValue = value;

  return ref;
}
function CreateNumberArrayReference(value){
  var ref;
  ref = {};
  ref.numberArray = value;

  return ref;
}
function CreateNumberArrayReferenceLengthValue(length, value){
  var ref;
  var i;
  ref = {};
  ref.numberArray = [];
  ref.numberArray.length = length;

  for(i = 0; i < length; i = i + 1){
    ref.numberArray[i] = value;
  }

  return ref;
}
function FreeNumberArrayReference(numberArrayReference){
  delete(numberArrayReference.numberArray);
  delete(numberArrayReference);
}
function CreateStringReference(value){
  var ref;
  ref = {};
  ref.string = value;

  return ref;
}
function CreateStringReferenceLengthValue(length, value){
  var ref;
  var i;
  ref = {};
  ref.string = [];
  ref.string.length = length;

  for(i = 0; i < length; i = i + 1){
    ref.string[i] = value;
  }

  return ref;
}
function FreeStringReference(stringReference){
  delete(stringReference.string);
  delete(stringReference);
}
function CreateStringArrayReference(strings){
  var ref;
  ref = {};
  ref.stringArray = strings;

  return ref;
}
function CreateStringArrayReferenceLengthValue(length, value){
  var ref;
  var i;
  ref = {};
  ref.stringArray = [];
  ref.stringArray.length = length;

  for(i = 0; i < length; i = i + 1){
    ref.stringArray[i] = CreateStringReference(value);
  }

  return ref;
}
function FreeStringArrayReference(stringArrayReference){
  var i;
  for(i = 0; i < stringArrayReference.stringArray.length; i = i + 1){
    delete(stringArrayReference.stringArray[i]);
  }
  delete(stringArrayReference.stringArray);
  delete(stringArrayReference);
}
function CreatePriorityQueueBTNumbers(){
  var q;

  q = {};
  q.heap = CreateDynamicArrayNumbers();

  return q;
}
function FreePriorityQueueBTNumbers(q){
  FreeDynamicArrayNumbers(q.heap);
  delete(q);
}
function PeekPriorityQueueBTNumbers(q, keyReference){
  var found;

  if( !IsEmptyPriorityQueueBTNumbers(q) ){
    keyReference.numberValue = DynamicArrayNumbersIndex(q.heap, 0);
    found = true;
  }else{
    found = false;
  }

  return found;
}
function InsertIntoPriorityQueueBTNumbers(q, key){
  DynamicArrayAddNumber(q.heap, key);

  if(SizeOfPriorityQueueBTNumbers(q) >= 2){
    SiftUpPriorityQueueBTNumbers(q, q.heap.length - 1);
  }
}
function PopPriorityQueueBTNumbers(q, keyReference){
  var found;

  found = PeekPriorityQueueBTNumbers(q, keyReference);

  if(found){
    DeleteTopPriorityQueueBTNumbers(q);
  }

  return found;
}
function DeleteTopPriorityQueueBTNumbers(q){
  var found;
  var last;

  found = IsEmptyPriorityQueueBTNumbers(q);

  if( !IsEmptyPriorityQueueBTNumbers(q) ){
    last = q.heap.length - 1;
    SwapElementsOfArray(q.heap.array, 0, last);

    DynamicArrayRemoveNumber(q.heap, last);

    SiftDownPriorityQueueBTNumbers(q, 0);
  }

  return found;
}
function ArrayToPriorityQueueBTNumbers(keys){
  var q;
  var i;

  q = CreatePriorityQueueBTNumbers();

  for(i = 0; i < keys.length; i = i + 1){
    InsertIntoPriorityQueueBTNumbers(q, keys[i]);
  }

  return q;
}
function SizeOfPriorityQueueBTNumbers(q){
  return q.heap.length;
}
function IsEmptyPriorityQueueBTNumbers(q){
  return q.heap.length == 0;
}
function SiftUpPriorityQueueBTNumbers(q, index){
  var parent;
  var iKey, pKey;
  var done;

  done = false;
  for(;  !done  && index != 0; ){
    parent = Math.floor((index - 1)/2);

    iKey = DynamicArrayNumbersIndex(q.heap, index);
    pKey = DynamicArrayNumbersIndex(q.heap, parent);

    if(iKey > pKey){
      SwapElementsOfArray(q.heap.array, index, parent);
    }else{
      done = true;
    }

    index = parent;
  }
}
function SiftDownPriorityQueueBTNumbers(q, index){
  var parent, c1, c2;
  var c1Key, c2Key, pKey, size;
  var done;

  size = SizeOfPriorityQueueBTNumbers(q);

  done = false;
  for(;  !done ; ){
    parent = index;
    c1 = 2*parent + 1;
    c2 = 2*parent + 2;

    pKey = DynamicArrayNumbersIndex(q.heap, parent);
    c1Key = DynamicArrayNumbersIndex(q.heap, c1);
    c2Key = DynamicArrayNumbersIndex(q.heap, c2);

    if(c1Key > pKey && c1 < size || c2Key > pKey && c2 < size){
      if(c1Key >= c2Key && c1 < size){
        SwapElementsOfArray(q.heap.array, c1, parent);
        index = c1;
      }else if(c1Key <= c2Key && c2 < size){
        SwapElementsOfArray(q.heap.array, c2, parent);
        index = c2;
      }else{
        done = true;
      }
    }else{
      done = true;
    }
  }
}
function CreatePriorityQueueBTNumKeyValue(){
  var q;

  q = {};
  q.heapKey = CreateDynamicArrayNumbers();
  q.heapValue = CreateDynamicArrayNumbers();

  return q;
}
function FreePriorityQueueBTNumKeyValue(q){
  FreeDynamicArrayNumbers(q.heapKey);
  FreeDynamicArrayNumbers(q.heapValue);
  delete(q);
}
function PeekPriorityQueueBTNumKeyValue(q, keyReference, valueReference){
  var found;

  if( !IsEmptyPriorityQueueBTNumKeyValue(q) ){
    keyReference.numberValue = DynamicArrayNumbersIndex(q.heapKey, 0);
    valueReference.numberValue = DynamicArrayNumbersIndex(q.heapValue, 0);
    found = true;
  }else{
    found = false;
  }

  return found;
}
function InsertIntoPriorityQueueBTNumKeyValue(q, key, value){
  DynamicArrayAddNumber(q.heapKey, key);
  DynamicArrayAddNumber(q.heapValue, value);

  if(SizeOfPriorityQueueBTNumKeyValue(q) >= 2){
    SiftUpPriorityQueueBTNumKeyValue(q, q.heapKey.length - 1);
  }
}
function PopPriorityQueueBTNumKeyValue(q, keyReference, valueReference){
  var found;

  found = PeekPriorityQueueBTNumKeyValue(q, keyReference, valueReference);

  if(found){
    DeleteTopPriorityQueueBTNumKeyValue(q);
  }

  return found;
}
function DeleteTopPriorityQueueBTNumKeyValue(q){
  var found;
  var last;

  found = IsEmptyPriorityQueueBTNumKeyValue(q);

  if( !IsEmptyPriorityQueueBTNumKeyValue(q) ){
    last = q.heapKey.length - 1;
    SwapElementsOfArray(q.heapKey.array, 0, last);
    SwapElementsOfArray(q.heapValue.array, 0, last);

    DynamicArrayRemoveNumber(q.heapKey, last);
    DynamicArrayRemoveNumber(q.heapValue, last);

    SiftDownPriorityQueueBTNumKeyValue(q, 0);
  }

  return found;
}
function ArrayToPriorityQueueBTNumKeyValue(keys, values){
  var q;
  var i;

  q = CreatePriorityQueueBTNumKeyValue();

  for(i = 0; i < keys.length; i = i + 1){
    InsertIntoPriorityQueueBTNumKeyValue(q, keys[i], values[i]);
  }

  return q;
}
function SizeOfPriorityQueueBTNumKeyValue(q){
  return q.heapKey.length;
}
function IsEmptyPriorityQueueBTNumKeyValue(q){
  return q.heapKey.length == 0;
}
function SiftUpPriorityQueueBTNumKeyValue(q, index){
  var parent;
  var iKey, pKey;
  var done;

  done = false;
  for(;  !done  && index != 0; ){
    parent = Math.floor((index - 1)/2);

    iKey = DynamicArrayNumbersIndex(q.heapKey, index);
    pKey = DynamicArrayNumbersIndex(q.heapKey, parent);

    if(iKey > pKey){
      SwapElementsOfArray(q.heapKey.array, index, parent);
      SwapElementsOfArray(q.heapValue.array, index, parent);
    }else{
      done = true;
    }

    index = parent;
  }
}
function SiftDownPriorityQueueBTNumKeyValue(q, index){
  var parent, c1, c2;
  var c1Key, c2Key, pKey, size;
  var done;

  size = SizeOfPriorityQueueBTNumKeyValue(q);

  c1Key = 0;
  c2Key = 0;
  done = false;
  for(;  !done ; ){
    parent = index;
    c1 = 2*parent + 1;
    c2 = 2*parent + 2;

    pKey = DynamicArrayNumbersIndex(q.heapKey, parent);
    if(c1 < size){
      c1Key = DynamicArrayNumbersIndex(q.heapKey, c1);
    }
    if(c2 < size){
      c2Key = DynamicArrayNumbersIndex(q.heapKey, c2);
    }

    if(c1Key > pKey && c1 < size || c2Key > pKey && c2 < size){
      if(c1Key >= c2Key && c1 < size){
        SwapElementsOfArray(q.heapKey.array, c1, parent);
        SwapElementsOfArray(q.heapValue.array, c1, parent);
        index = c1;
      }else if(c1Key <= c2Key && c2 < size){
        SwapElementsOfArray(q.heapKey.array, c2, parent);
        SwapElementsOfArray(q.heapValue.array, c2, parent);
        index = c2;
      }else{
        done = true;
      }
    }else{
      done = true;
    }
  }
}
function CreateLinkedListNumbers(){
  var ll;

  ll = {};
  ll.first = {};
  ll.last = ll.first;
  ll.last.end = true;

  return ll;
}
function CreateLinkedListNumbersArray(length){
  var lls;
  var i;

  lls = [];
  lls.length = length;
  for(i = 0; i < lls.length; i = i + 1){
    lls[i] = CreateLinkedListNumbers();
  }

  return lls;
}
function LinkedListAddNumber(ll, value){
  ll.last.end = false;
  ll.last.value = value;
  ll.last.next = {};
  ll.last.next.end = true;
  ll.last = ll.last.next;
}
function LinkedListNumbersLength(ll){
  var l;
  var node;

  l = 0;
  node = ll.first;
  for(;  !node.end ; ){
    node = node.next;
    l = l + 1;
  }

  return l;
}
function LinkedListNumbersIndex(ll, index){
  var i;
  var node;

  node = ll.first;
  for(i = 0; i < index; i = i + 1){
    node = node.next;
  }

  return node.value;
}
function LinkedListInsertNumber(ll, index, value){
  var i;
  var node, tmp;

  if(index == 0){
    tmp = ll.first;
    ll.first = {};
    ll.first.next = tmp;
    ll.first.value = value;
    ll.first.end = false;
  }else{
    node = ll.first;
    for(i = 0; i < index - 1; i = i + 1){
      node = node.next;
    }

    tmp = node.next;
    node.next = {};
    node.next.next = tmp;
    node.next.value = value;
    node.next.end = false;
  }
}
function LinkedListSet(ll, index, value){
  var i;
  var node;

  node = ll.first;
  for(i = 0; i < index; i = i + 1){
    node = node.next;
  }

  node.next.value = value;
}
function LinkedListRemoveNumber(ll, index){
  var i;
  var node, prev;

  node = ll.first;
  prev = ll.first;

  for(i = 0; i < index; i = i + 1){
    prev = node;
    node = node.next;
  }

  if(index == 0){
    ll.first = prev.next;
  }

  prev.next = prev.next.next;
}
function FreeLinkedListNumbers(ll){
  var node, prev;

  node = ll.first;

  for(;  !node.end ; ){
    prev = node;
    node = node.next;
    delete(prev);
  }

  delete(node);
}
function FreeLinkedListNumbersArray(lls){
  var i;

  for(i = 0; i < lls.length; i = i + 1){
    FreeLinkedListNumbers(lls[i]);
  }
  delete(lls);
}
function LinkedListNumbersToArray(ll){
  var array;
  var length, i;
  var node;

  node = ll.first;

  length = LinkedListNumbersLength(ll);

  array = [];
  array.length = length;

  for(i = 0; i < length; i = i + 1){
    array[i] = node.value;
    node = node.next;
  }

  return array;
}
function ArrayToLinkedListNumbers(array){
  var ll;
  var i;

  ll = CreateLinkedListNumbers();

  for(i = 0; i < array.length; i = i + 1){
    LinkedListAddNumber(ll, array[i]);
  }

  return ll;
}
function LinkedListNumbersEqual(a, b){
  var equal, done;
  var an, bn;

  an = a.first;
  bn = b.first;

  equal = true;
  done = false;
  for(; equal &&  !done ; ){
    if(an.end == bn.end){
      if(an.end){
        done = true;
      }else if(an.value == bn.value){
        an = an.next;
        bn = bn.next;
      }else{
        equal = false;
      }
    }else{
      equal = false;
    }
  }

  return equal;
}
function CreateDynamicArrayNumbers(){
  var da;

  da = {};
  da.array = [];
  da.array.length = 10;
  da.length = 0;

  return da;
}
function CreateDynamicArrayNumbersWithInitialCapacity(capacity){
  var da;

  da = {};
  da.array = [];
  da.array.length = capacity;
  da.length = 0;

  return da;
}
function DynamicArrayAddNumber(da, value){
  if(da.length == da.array.length){
    DynamicArrayNumbersIncreaseSize(da);
  }

  da.array[da.length] = value;
  da.length = da.length + 1;
}
function DynamicArrayNumbersIncreaseSize(da){
  var newLength, i;
  var newArray;

  newLength = Math.round(da.array.length*3/2);
  newArray = [];
  newArray.length = newLength;

  for(i = 0; i < da.array.length; i = i + 1){
    newArray[i] = da.array[i];
  }

  delete(da.array);

  da.array = newArray;
}
function DynamicArrayNumbersDecreaseSizeNecessary(da){
  var needsDecrease;

  needsDecrease = false;

  if(da.length > 10){
    needsDecrease = da.length <= Math.round(da.array.length*2/3);
  }

  return needsDecrease;
}
function DynamicArrayNumbersDecreaseSize(da){
  var newLength, i;
  var newArray;

  newLength = Math.round(da.array.length*2/3);
  newArray = [];
  newArray.length = newLength;

  for(i = 0; i < da.array.length; i = i + 1){
    newArray[i] = da.array[i];
  }

  delete(da.array);

  da.array = newArray;
}
function DynamicArrayNumbersIndex(da, index){
  return da.array[index];
}
function DynamicArrayNumbersLength(da){
  return da.length;
}
function DynamicArrayInsertNumber(da, index, value){
  var i;

  if(da.length == da.array.length){
    DynamicArrayNumbersIncreaseSize(da);
  }

  for(i = da.length; i >= index; i = i - 1){
    da.array[i + 1] = da.array[i];
  }

  da.array[index] = value;

  da.length = da.length + 1;
}
function DynamicArraySet(da, index, value){
  da.array[index] = value;
}
function DynamicArrayRemoveNumber(da, index){
  var i;

  for(i = index; i < da.length - 1; i = i + 1){
    da.array[i] = da.array[i + 1];
  }

  da.length = da.length - 1;

  if(DynamicArrayNumbersDecreaseSizeNecessary(da)){
    DynamicArrayNumbersDecreaseSize(da);
  }
}
function FreeDynamicArrayNumbers(da){
  delete(da.array);
  delete(da);
}
function DynamicArrayNumbersToArray(da){
  var array;
  var i;

  array = [];
  array.length = da.length;

  for(i = 0; i < da.length; i = i + 1){
    array[i] = da.array[i];
  }

  return array;
}
function ArrayToDynamicArrayNumbersWithOptimalSize(array){
  var da;
  var i;
  var c, n, newCapacity;

  /*
         c = 10*(3/2)^n
         log(c) = log(10*(3/2)^n)
         log(c) = log(10) + log((3/2)^n)
         log(c) = 1 + log((3/2)^n)
         log(c) - 1 = log((3/2)^n)
         log(c) - 1 = n*log(3/2)
         n = (log(c) - 1)/log(3/2)
         */
  c = array.length;
  n = (Math.log(c) - 1)/Math.log(3/2);
  newCapacity = Math.floor(n) + 1;

  da = CreateDynamicArrayNumbersWithInitialCapacity(newCapacity);

  for(i = 0; i < array.length; i = i + 1){
    da.array[i] = array[i];
  }

  return da;
}
function ArrayToDynamicArrayNumbers(array){
  var da;

  da = {};
  da.array = CopyNumberArray(array);
  da.length = array.length;

  return da;
}
function DynamicArrayNumbersEqual(a, b){
  var equal;
  var i;

  equal = true;
  if(a.length == b.length){
    for(i = 0; i < a.length && equal; i = i + 1){
      if(a.array[i] != b.array[i]){
        equal = false;
      }
    }
  }else{
    equal = false;
  }

  return equal;
}
function DynamicArrayNumbersToLinkedList(da){
  var ll;
  var i;

  ll = CreateLinkedListNumbers();

  for(i = 0; i < da.length; i = i + 1){
    LinkedListAddNumber(ll, da.array[i]);
  }

  return ll;
}
function LinkedListToDynamicArrayNumbers(ll){
  var da;
  var i;
  var node;

  node = ll.first;

  da = {};
  da.length = LinkedListNumbersLength(ll);

  da.array = [];
  da.array.length = da.length;

  for(i = 0; i < da.length; i = i + 1){
    da.array[i] = node.value;
    node = node.next;
  }

  return da;
}
function AddNumber(list, a){
  var newlist;
  var i;

  newlist = [];
  newlist.length = list.length + 1;
  for(i = 0; i < list.length; i = i + 1){
    newlist[i] = list[i];
  }
  newlist[list.length] = a;

  delete(list);

  return newlist;
}
function AddNumberRef(list, i){
  list.numberArray = AddNumber(list.numberArray, i);
}
function RemoveNumber(list, n){
  var newlist;
  var i;

  newlist = [];
  newlist.length = list.length - 1;

  if(n >= 0 && n < list.length){
    for(i = 0; i < list.length; i = i + 1){
      if(i < n){
        newlist[i] = list[i];
      }
      if(i > n){
        newlist[i - 1] = list[i];
      }
    }

    delete(list);
  }else{
    delete(newlist);
  }

  return newlist;
}
function GetNumberRef(list, i){
  return list.numberArray[i];
}
function RemoveNumberRef(list, i){
  list.numberArray = RemoveNumber(list.numberArray, i);
}
function AddString(list, a){
  var newlist;
  var i;

  newlist = [];
  newlist.length = list.length + 1;

  for(i = 0; i < list.length; i = i + 1){
    newlist[i] = list[i];
  }
  newlist[list.length] = a;

  delete(list);

  return newlist;
}
function AddStringRef(list, i){
  list.stringArray = AddString(list.stringArray, i);
}
function RemoveString(list, n){
  var newlist;
  var i;

  newlist = [];
  newlist.length = list.length - 1;

  if(n >= 0 && n < list.length){
    for(i = 0; i < list.length; i = i + 1){
      if(i < n){
        newlist[i] = list[i];
      }
      if(i > n){
        newlist[i - 1] = list[i];
      }
    }

    delete(list);
  }else{
    delete(newlist);
  }

  return newlist;
}
function GetStringRef(list, i){
  return list.stringArray[i];
}
function RemoveStringRef(list, i){
  list.stringArray = RemoveString(list.stringArray, i);
}
function AddBoolean(list, a){
  var newlist;
  var i;

  newlist = [];
  newlist.length = list.length + 1;
  for(i = 0; i < list.length; i = i + 1){
    newlist[i] = list[i];
  }
  newlist[list.length] = a;

  delete(list);

  return newlist;
}
function AddBooleanRef(list, i){
  list.booleanArray = AddBoolean(list.booleanArray, i);
}
function RemoveBoolean(list, n){
  var newlist;
  var i;

  newlist = [];
  newlist.length = list.length - 1;

  if(n >= 0 && n < list.length){
    for(i = 0; i < list.length; i = i + 1){
      if(i < n){
        newlist[i] = list[i];
      }
      if(i > n){
        newlist[i - 1] = list[i];
      }
    }

    delete(list);
  }else{
    delete(newlist);
  }

  return newlist;
}
function GetBooleanRef(list, i){
  return list.booleanArray[i];
}
function RemoveDecimalRef(list, i){
  list.booleanArray = RemoveBoolean(list.booleanArray, i);
}
function AddCharacter(list, a){
  var newlist;
  var i;

  newlist = [];
  newlist.length = list.length + 1;
  for(i = 0; i < list.length; i = i + 1){
    newlist[i] = list[i];
  }
  newlist[list.length] = a;

  delete(list);

  return newlist;
}
function AddCharacterRef(list, i){
  list.string = AddCharacter(list.string, i);
}
function RemoveCharacter(list, n){
  var newlist;
  var i;

  newlist = [];
  newlist.length = list.length - 1;

  if(n >= 0 && n < list.length){
    for(i = 0; i < list.length; i = i + 1){
      if(i < n){
        newlist[i] = list[i];
      }
      if(i > n){
        newlist[i - 1] = list[i];
      }
    }

    delete(list);
  }else{
    delete(newlist);
  }

  return newlist;
}
function GetCharacterRef(list, i){
  return list.string[i];
}
function RemoveCharacterRef(list, i){
  list.string = RemoveCharacter(list.string, i);
}
function TreeHeight(tree){
  var height, i, branchHeight;
  var heightSet;

  heightSet = false;
  height = 0;

  for(i = 0; i < tree.branches.length; i = i + 1){
    branchHeight = TreeHeight(tree.branches[i]);
    if( !heightSet ){
      height = branchHeight;
      heightSet = true;
    }else if(branchHeight > height){
      height = branchHeight;
    }
  }

  if(tree.branches.length == 0){
    height = 0;
  }else{
    height = height + 1;
  }

  return height;
}
function TreeNumberOfNodes(tree){
  var nodes, i;

  nodes = 0;

  for(i = 0; i < tree.branches.length; i = i + 1){
    nodes = nodes + TreeNumberOfNodes(tree.branches[i]);
  }

  return nodes + 1;
}
function AssertFalse(b, failures){
  if(b){
    failures.numberValue = failures.numberValue + 1;
  }
}
function AssertTrue(b, failures){
  if( !b ){
    failures.numberValue = failures.numberValue + 1;
  }
}
function AssertEquals(a, b, failures){
  if(a != b){
    failures.numberValue = failures.numberValue + 1;
  }
}
function AssertBooleansEqual(a, b, failures){
  if(a != b){
    failures.numberValue = failures.numberValue + 1;
  }
}
function AssertCharactersEqual(a, b, failures){
  if(a != b){
    failures.numberValue = failures.numberValue + 1;
  }
}
function AssertStringEquals(a, b, failures){
  if( !StringsEqual(a, b) ){
    failures.numberValue = failures.numberValue + 1;
  }
}
function AssertNumberArraysEqual(a, b, failures){
  var i;

  if(a.length == b.length){
    for(i = 0; i < a.length; i = i + 1){
      AssertEquals(a[i], b[i], failures);
    }
  }else{
    failures.numberValue = failures.numberValue + 1;
  }
}
function AssertBooleanArraysEqual(a, b, failures){
  var i;

  if(a.length == b.length){
    for(i = 0; i < a.length; i = i + 1){
      AssertBooleansEqual(a[i], b[i], failures);
    }
  }else{
    failures.numberValue = failures.numberValue + 1;
  }
}
function AssertStringArraysEqual(a, b, failures){
  var i;

  if(a.length == b.length){
    for(i = 0; i < a.length; i = i + 1){
      AssertStringEquals(a[i].string, b[i].string, failures);
    }
  }else{
    failures.numberValue = failures.numberValue + 1;
  }
}
