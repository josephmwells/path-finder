#include "maze.h"
#include "path.h"
#include<queue>
#include<vector>
#include<list>
#include<tuple>
#include<utility>
#include<iostream>
#include<climits>
#include<sstream>

#include<stack>
#include<algorithm>
#include<map>
using namespace std;

path solve_dfs(Maze& m, int rows, int cols);
path solve_bfs(Maze& m, int rows, int cols);
path solve_dijkstra(Maze& m, int rows, int cols);
path solve_tour(Maze& m, int rows, int cols);

bool have_visited(const point& pos, const int dir, const vector<point>& visited);


int main(int argc, char** argv)
{
    if(argc != 4)
    {
        cerr << "usage:\n"
             << "./maze option rows cols\n"
             << " options:\n"
             << "  -dfs: depth first search (backtracking)\n"
             << "  -bfs: breadth first search\n"
             << "  -dij: dijkstra's algorithm\n"
             << "  -tour: all corners tour\n"
             << "  -basic: run dfs, bfs, and dij\n"
             << "  -advanced: run dfs, bfs, dij and tour" << endl;
        return 0;
    }
    string opt(argv[1]);

    int rows, cols;
    stringstream s;
    s << argv[2] << " " << argv[3];
    s >> rows >> cols;

    // construct a new random maze;
    Maze m(rows, cols);

    // print the initial maze out
    cout << "Initial maze" << endl;
    m.print_maze(cout, opt == "-dij" || opt == "-tour");

    if(opt == "-dfs")
    {
        cout << "\nSolved dfs" << endl;
        path p = solve_dfs(m, rows, cols);
        m.print_maze_with_path(cout, p, false, false);
    }


    if(opt == "-bfs")
    {
        cout << "\nSolved bfs" << endl;
        path p = solve_bfs(m, rows, cols);
        m.print_maze_with_path(cout, p, false, false);
    }

    if(opt == "-dij")
    {
        cout << "\nSolved dijkstra" << endl;
        path p = solve_dijkstra(m, rows, cols);
        m.print_maze_with_path(cout, p, true, false);
    }

    if(opt == "-tour")
    {
        cout << "\nSolved all courners tour" << endl;
        path p = solve_tour(m, rows, cols);
        m.print_maze_with_path(cout, p, true, true);
    }
    if(opt == "-basic")
    {
        cout << "\nSolved dfs" << endl;
        path p = solve_dfs(m, rows, cols);
        m.print_maze_with_path(cout, p, false, false);

        cout << "\nSolved bfs" << endl;
        p = solve_bfs(m, rows, cols);
        m.print_maze_with_path(cout, p, false, false);

        cout << "\nSolved dijkstra" << endl;
        p = solve_dijkstra(m, rows, cols);
        m.print_maze_with_path(cout, p, true, false);
    }
    if(opt == "-advanced")
    {
        cout << "\nSolved dfs" << endl;
        path p = solve_dfs(m, rows, cols);
        m.print_maze_with_path(cout, p, false, false);

        cout << "\nSolved bfs" << endl;
        p = solve_bfs(m, rows, cols);
        m.print_maze_with_path(cout, p, false, false);

        cout << "\nSolved dijkstra" << endl;
        p = solve_dijkstra(m, rows, cols);
        m.print_maze_with_path(cout, p, true, false);

        cout << "\nSolved all courners tour" << endl;
        p = solve_tour(m, rows, cols);
        m.print_maze_with_path(cout, p, true, true);
    }
}

bool have_visited(const point& pos, const int dir, const vector<point>& visited)
{
  // Find if a position has been visited using std::find looking for the
  // position in the given direction
  if(find(visited.begin(), visited.end(), pos + moveIn(dir)) != visited.end())
    return true;
  else
    return false;
}

path solve_dfs(Maze& m, int rows, int cols)
{
  point pos(0, 0);
  int dir;
  vector<point> visited;
  stack<point> cur_path;

  cur_path.push(pos);
  visited.push_back(pos);

  cout << "Entering maze";
  // keep running until the current position is at the end of the maze
  while(pos.first != (rows - 1) || pos.second != (cols - 1)) {
    // check if we can move to the next position and assign direction 
    if(m.can_go_down(pos.first, pos.second) && !have_visited(pos, DOWN, visited)) {
      dir = DOWN;
    } else if(m.can_go_left(pos.first, pos.second) && !have_visited(pos, LEFT, visited)) {
      dir = LEFT;
    } else if(m.can_go_up(pos.first, pos.second) && !have_visited(pos, UP, visited)) {
      dir = UP;
    } else if(m.can_go_right(pos.first, pos.second) && !have_visited(pos, RIGHT, visited)) {
      dir = RIGHT;
    } else {
      dir = FAIL;
    }
    // If we can't move, go back, otherwise move in assigned direction
    if(dir == FAIL) {
      cur_path.pop();
      pos = cur_path.top();
    } else {
      pos = pos + moveIn(dir);    
      cur_path.push(pos);
      visited.push_back(pos);
    }
  }

  // Copy the stack into a list
  path points;
  while(!cur_path.empty()) {
    points.push_front(cur_path.top());
    cur_path.pop();
  }

  return points;
}



path solve_bfs(Maze& m, int rows, int cols)
{
  vector<point> visited;
  queue<point> q;
  map<point, point> parent;

  point current(0, 0);

  q.push(current);
  visited.push_back(current);

  bool end = false;
  while(!q.empty() && !end) {
    current = q.front();
    q.pop();
    
    // Add neighbors to queue if valid, mark as visited, and add to parent tree
    for (int dir = 0; dir < 4; ++dir) {
      if(m.can_go(dir, current.first, current.second) && !have_visited(current, dir, visited)) {
        point neighbor = current + moveIn(dir);
        q.push(neighbor);
        visited.push_back(neighbor);
        parent[neighbor] = current;

        if(neighbor == point(rows-1, cols-1))
          end = true;
      }
    }
  }
  
  path points;

  // Iterate back through each parent, starting at the end node
  // then reverse the vector so that it is in the right order
  for(point cur = point(rows-1, cols-1); cur != point(0, 0); cur = parent[cur]) {
    points.push_back(cur);
  }
  points.push_back(point(0, 0));
  reverse(points.begin(), points.end());

  return points;
}

struct Cell {
  point p;
  int cost;

  Cell(point p, int cost) : p(p), cost(cost) {}
};

struct CompareCost  {
  bool operator()(Cell const& p1, Cell const& p2)
  {
    return p1.cost > p2.cost;
  }
};

path solve_dijkstra(Maze& m, int rows, int cols)
{
  vector<point> visited;
  priority_queue<Cell, vector<Cell>, CompareCost> q;
  map<point ,point> parent;

  point current(0, 0);

  q.push(Cell(current, 0));
  visited.push_back(current);

  bool end = false;
  while(!q.empty() && !end) {
    current = q.top().p;
    q.pop();

    // Add neighbors to queue if valid, mark as visited, and add to parent tree
    for (int dir = 0; dir < 4; ++dir) {
      if(m.can_go(dir, current.first, current.second) && !have_visited(current, dir, visited)) {
        point neighbor = current + moveIn(dir);
        q.push(Cell(neighbor, m.cost(current.first, current.second, dir)));
        visited.push_back(neighbor);
        parent[neighbor] = current;

        if (neighbor == point(rows-1, cols-1))
          end = true;
      }
    }
  }

  path points;

  // Iterate back through each parent, starting at the end node
  // then reverse the vector so that it is in the right order
  for(point cur = point(rows-1, cols-1); cur != point(0,0); cur = parent[cur]) {
    points.push_back(cur);
  }
  points.push_back(point(0, 0));
  reverse(points.begin(), points.end());

  return points;
}

path solve_tour(Maze& m, int rows, int cols)
{
    return list<point>();
}

