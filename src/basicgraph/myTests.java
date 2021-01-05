package basicgraph;

import util.GraphLoader;

public class myTests {

    private void printMatrix (int[][] matrix){
        System.out.println ("\n Imprimiendo matrix: \n");
        for (int i= 0 ; i < matrix.length ; i++){
            System.out.println ("\n ");
            for (int j=0 ; j < matrix.length; j++){
                System.out.println (" " + matrix[i][j]);
            }

        }
    }
    public static void main (String[] args) {
        GraphLoader.createIntersectionsFile("data/testdata/simpletest.map", "data/intersections/ucsd.intersections");
        // For testing of Part 1 functionality
        // Add your tests here to make sure your degreeSequence method is returning
        // the correct list, after examining the graphs.
        System.out.println("Loading graphs based on real data...");
        System.out.println("Roads / intersections:");
        //GraphAdjList graphFromFile = new GraphAdjList();
        GraphAdjMatrix graphFromFile = new GraphAdjMatrix();
        GraphLoader.loadRoadMap("data/testdata/simpletest.map", graphFromFile);
        System.out.println(graphFromFile);
        System.out.println("\n****");
        System.out.println("\n Degree sequence: \n");
        System.out.println(graphFromFile.degreeSequence());


        // You can test with real road data here.  Use the data files in data/maps

    }
}
