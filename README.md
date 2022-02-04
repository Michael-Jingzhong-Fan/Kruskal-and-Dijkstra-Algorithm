# Kruskal-and-Dijkstra-Algorithm

The widest path problem also known as the maximum capacity problem asks you to find the path with the
greatest minimum edge weight. Our problem is to find the path with the smallest minimum edge weight.
The widest path can be found by finding the path between two nodes in the maximum spanning tree of a
graph. I considered that perhaps I can find the 'narrowest' path that our problem requires by finding
the path between two nodes in the minimum spanning tree. With some case by case testing on paper,
looking for a counter example and some consideration to be sure, I decided to try this.

I now knew I had to form a completed graph of all nodes, form a minimum spanning tree and then find
the path connecting two using a search algorithm. Immediately, Kruskal's Algorithm for finding a
minimum spanning tree and Dijkstra's Algorithm for finding the path come to mind as these are the 
algorithms I am most familiar with. To verify that these algorithms will be near optimal, I discover
Prim's algorithm which I am less familiar with may also do the job, however, with some calculation,
I can see it has a time complexity of O(V2) which is much greater than the time complexity O(E log V)
where E is the number of vertices which Kruskal's Algorithm has.I notably use a dictionary to store 
the edges and weights and take advantage of hashing for faster look up time. I add sanity checks to 
check our input is valid.

There are some improvements that can likely be made. I have not been extremely careful with the 
datastructure that I pick throughout the code. Perhaps, a list here or there could be replaced 
with a dictionary for faster lookup times. Additionally, I could implement a fibonacci heap and 
improve Prim's algorithm to a time complexity of O(Elog(V)) where V is the number of vertices. 
I could then test this against Kruskal's algorithm. Consensus is that Prim's algorithm performs 
better on dense graphs (such as ours when we are finding the minimum spanning tree) so, there may 
be an improvement in performance. Documentation comments also need to be made consistent throughout. 
