# Operation research project

This program compute : 
- Max flow and the list of flow value traversing each arc
- Min cut and the list of arc that form the minimum cut

By default, this program compute the max flow on the graph in data/data.txt.
If you want to work with another graph, you can give the absolute path to file 
as first argument in the command line or change the code to use your graph (@deprecated).

### Important : 
Each node with capacity = 0 is ignored.