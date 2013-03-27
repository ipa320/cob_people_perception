/*

This file is part of The Kuhn-Munkres Assignment Algorithm.

The Kuhn-Munkres Assignment Algorithm is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

The Kuhn-Munkres Assignment Algorithm is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Foobar.  If not, see <http://www.gnu.org/licenses/>.

*/




This is an implementation of the Kuhn-Munkres , or Hungarian, assignment algorithm.
Basic information about the algorithm and the problems it is used to solve can be found at 
http://en.wikipedia.org/wiki/Hungarian_algorithm
This particular implementation solves the minimization problem.

The munkres class is declared with no parameters.
To load a matrix of weights you must call the function load_weights() which takes a single parameter of type vector< vector<int> >.
To find an optimal assignment you must then call assign() which returns the total cost of the assignment as an int and takes a pointer
to an array of objects of type ordered_pair.  This array should be of length min((number of rows), (number of columns)).
For a diagnostic output of each step of the algorithm, call the function set_diag with a bool parameter of true.

The main.cpp included with the class includes a pseudo-random test-case generator.  To vary the sample set simply change the 
global variables min_size, max_size, min_weight, max_weight.
There is no error checking to make sure that you're mins are actually less than your maxes, but it won't break if they aren't.
If you want a console output of the sample matrix, set display_matrix to true.


-Ryan Rigdon