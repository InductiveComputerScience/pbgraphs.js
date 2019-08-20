var fs = require('fs');

eval(fs.readFileSync('DirectedGraphs.js')+'');

var directedGraph = {
   "nodes": [
    {
     "edge": [
      {"nodeNr": 1,"weight": 7},
      {"nodeNr": 2,"weight": 9},
      {"nodeNr": 5,"weight": 14}
     ]
    },
    {
     "edge": [
      {"nodeNr": 0,"weight": 7},
      {"nodeNr": 2,"weight": 10},
      {"nodeNr": 3,"weight": 15}
     ]
    },
    {
     "edge": [
      {"nodeNr": 0,"weight": 9},
      {"nodeNr": 1,"weight": 10},
      {"nodeNr": 3,"weight": 11},
      {"nodeNr": 5,"weight": 2}
     ]
    },
    {
     "edge": [
      {"nodeNr": 1,"weight": 15},
      {"nodeNr": 2,"weight": 11},
      {"nodeNr": 4,"weight": 6}
     ]
    },
    {
     "edge": [
      {"nodeNr": 3,"weight": 6},
      {"nodeNr": 5,"weight": 9}
     ]
    },
    {
     "edge": [
      {"nodeNr": 0,"weight": 14},
      {"nodeNr": 2,"weight": 2},
      {"nodeNr": 4,"weight": 9}
     ]
    }
   ]
  };

var d = {};
var p = {};
var ds = {};
DijkstrasAlgorithm(directedGraph, 0, d, ds, p);

console.log(d);
