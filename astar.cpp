#include <bits/stdc++.h>


using namespace std;

//Describes the information in the individual cells of the grid
struct node
{
public:

	double cost = numeric_limits<int>::max();

	//coordinates of the parent of the cell which is part of the final path
	int parent[2] = {-numeric_limits<int>::max(),-numeric_limits<int>::max()};
    
    //coordinates of the cell in the grid
	int x=0;
	int y = 0;
};

vector <node> trace_path(node current, vector<vector<node>> grid)
{
    vector<node> path;
    while(current.parent[0]!=-1 && current.parent[1]!=-1)
    {
     path.push_back(current);
     current = grid[current.parent[0]][current.parent[1]];   
    }
    path.push_back(current);
    return path;
}

//calculates the euclidian distance between two cells in the grid
double dist(node parent,node child)
{	
	double distance  = sqrt(pow((parent.x-child.x),2)+pow((parent.y-child.y),2));
	return(distance);
}

//Checks if the cell has an obstacle or no
bool is_obstacle(node cell, vector<vector<int>> map)
{
	if(map[cell.x][cell.y]==1)
		return true;
	return false;
}

//Finds the cell with the least cost from all the unvisited nodes in queue
int find_min(vector<node> unvisited_queue,node goal)
{	
	node temp_node;
	int temp_cost = 9999;

	//the index of the node in the unvisited vector
	int index = 0;
	for(int i=0;i<unvisited_queue.size();i++)
	{	
		double cost = unvisited_queue[i].cost+dist(unvisited_queue[i],goal);
		if(cost<temp_cost)
		{
			temp_node = unvisited_queue[i];
			temp_cost = cost;
			index = i;
		}
	}
	return index;
}

//Searches if the 'cell' is in the 'vec'. Returns false if its absent.
bool is_in_vector(vector<node> vec, node cell)
{
	for(int i=0;i<vec.size();i++)
	{
		if(vec[i].x == cell.x && vec[i].y == cell.y)
			return true;
	}
	return false;
}

//performs A* 
vector<node> find_path(int startx, int starty, int goalx, int goaly, vector<vector<int>>map, vector<vector<node>> grid)
{

    node start;
	start.x = startx;
	start.y = starty;
	start.cost = 0;
    start.parent[0] = -1;
    start.parent[1] = -1;
    grid[startx][starty] = start;

	node goal;
	goal.x = goalx;
	goal.y = goaly;

	int width = grid.size();
	int height = grid[0].size();

	node current = start;

	double cost = dist(current,goal);	//cost to goal, stop when cost == 0

    vector <node> queue;
    vector <node> visited;
    visited.push_back(start);

	while(cost!=0)	
	{	
		
		int valid_neighbours = 0;
		for(int i=-1;i<2;i++)
		{
			int newX = current.x + i;

			for(int j=-1;j<2;j++)
			{ 
				if(!(i==0&&j==0))
				{int newY = current.y + j;
					if(newX>=0 && newX <width && newY>=0 && newY<height)
					{
						if(!is_obstacle(grid[newX][newY],map))	//check for collision
						{
								double newcost = current.cost + dist(current,grid[newX][newY]);	//compute cost to the neighbour from current cell

								valid_neighbours++;
								node neighbour = grid[newX][newY];

								//push the neighbour if it doesnt exist in the unvisited queue and the visited queue
								if(!is_in_vector(visited,neighbour) && !is_in_vector(queue,neighbour))
								{	
									neighbour.parent[0] = current.x;
									neighbour.parent[1] = current.y; 
									neighbour.cost = newcost;
									grid[newX][newY] = neighbour;
									queue.push_back(grid[newX][newY]);
								}

								//rewire in case the cost from current cell to a neighbour is less that the neighbours current cost
								if(newcost<=grid[newX][newY].cost)	
								{	
									
									neighbour.parent[0] = current.x;
									neighbour.parent[1] = current.y; 
									neighbour.cost = newcost;
									grid[newX][newY] = neighbour;
								}
							

						 }
					}
				}
				
			}
		}

		//find neighbour with minimum cost
		int index = find_min(queue,goal);

		node next = queue[index];
		queue.erase(queue.begin()+index);	//erase the cell from unvisited
		current = next;
	
		visited.push_back(current);		//push the cell into visited

		cost = dist(current,goal);		//compute cost to goal to stop when cost ==0

		
	}

    vector<node> path = trace_path(current,grid);
    return path;
}

int main()
{
	//input the data 

	//the map 
    std::vector<std::vector<int> > map{{ 0, 1, 0, 0, 0, 0},
                                       { 0, 1, 0, 0, 0, 0},
                                       { 0, 1, 0, 0, 0, 0},
                                       { 0, 1, 0, 0, 0, 0},
                                       { 0, 1, 0, 0, 0, 0},
                                       { 0, 0, 0, 0, 0, 0}};


    //starting and goal co-ordinates                            
    int startx = 0;
    int starty = 0;
    int goalx = 0;
    int goaly = 5;
 
    vector< vector<node>> grid;
    
    //initialize the grid
    for(int i=0;i<map.size();i++)
    { 
    	vector <node> row;
        for(int j=0;j<map[i].size();j++)
        {
           node temp;
           temp.x = i;
           temp.y = j;
           row.push_back(temp);
        }
     	grid.push_back(row);
    }
    
    vector <node> path;
    
    
	path = find_path(startx,  starty,  goalx,  goaly,  map,  grid);

	cout<<"FINAL PATH=";
	for(int i=path.size()-1;i>=0;i--)
	{
		cout<<" ("<<path[i].x<<","<<path[i].y<<")"<< " to ";
	}
    cout<<endl<<"COST = "<<path[0].cost<<endl;
    return 0;
}