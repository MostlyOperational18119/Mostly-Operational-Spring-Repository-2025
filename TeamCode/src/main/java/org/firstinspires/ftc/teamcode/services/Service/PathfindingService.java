package org.firstinspires.ftc.teamcode.services.Service;


import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.pathgen.Point;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;

public class PathfindingService {
    private Follower follower;
    private PoseUpdater poseUpdater;
    private List<Node> openSet ;
    private HashSet<Node> closedSet;

    private static final int BLACKLIST_MIN_X = 32;
    private static final int BLACKLIST_MIN_Y = 32;
    private static final int BLACKLIST_MAX_X = 112;
    private static final int BLACKLIST_MAX_Y = 112;

    private static final int[][] DIRECTIONS = {
            // 4 cardinal directions
            {1, 0},
            {0, 1},
            {-1, 0},
            {0, -1},

            // 4 diagonal directions
            {1, 1},
            {-1, 1},
            {1, -1},
            {-1, -1}
    };

    public PathfindingService (Follower follower, PoseUpdater poseUpdater) {
        this.follower = follower;
        this.poseUpdater = poseUpdater;
    }

    public static class Node {
        public int x, y, gCost, hCost, fCost;
        public Node parent;

        public Node(int x, int y) {
            this.x = x;
            this.y = y;
        }

        @Override
        public boolean equals(Object obj) {
            if (this == obj) return true;
            if (!(obj instanceof Node)) return false;
            Node node = (Node) obj;
            return x == node.x && y == node.y;
        }

        @Override
        public int hashCode() {
            return x * 31 + y;
        }
    }

    public List<Node> findPath(Node start, Node end, int gridSize) {
        openSet = new ArrayList<>();
        closedSet = new HashSet<>();
        start = roundToNearestPoint(start, gridSize);
        end = roundToNearestPoint(end, gridSize);

        start.gCost = 0;
        start.hCost = calculateHeuristic(start, end);
        start.fCost = start.gCost + start.hCost;
        start.parent = null;

        openSet.add(start);

        while (!openSet.isEmpty()) {
            openSet.sort((a, b) ->{
                if (a.fCost != b.fCost) {
                    return Integer.compare(a.fCost, b.fCost);
                }
                return Integer.compare(a.hCost, b.hCost);
            });

            Node currentNode = openSet.remove(0);

            if (currentNode.equals(end)) {
                return reconstructPath(currentNode);
            }

            closedSet.add(currentNode);

            processNeighbors(currentNode, end, gridSize);
        }
        return null;
    }

    private Node roundToNearestPoint(Node input, int gridSize) {
        return new Node(
                Math.round((float) input.x / gridSize) * gridSize,
                Math.round((float) input.y / gridSize) *gridSize
        );
    }

    private void processNeighbors(Node currentNode, Node end, int gridSize) {
        for (int[] direction : DIRECTIONS) {
            int neighborX = currentNode.x + (direction[0] * gridSize);
            int neighborY = currentNode.y + (direction[1] * gridSize);
            Node neighbor = new Node(neighborX, neighborY);

            boolean isTraversable = nodeIsTraversable(neighbor);

            if (!isTraversable || closedSet.contains(neighbor)) {
                continue;
            }

            double distance = calculateDistance(currentNode, neighbor);
            int costToNeighbor = (int)(currentNode.gCost + distance);

            Node existingNeighbor = findNodeInOpenSet(neighbor);

            int neighborGCost = (existingNeighbor != null) ? existingNeighbor.gCost : 0;

            if (costToNeighbor < neighborGCost || existingNeighbor == null) {
                Node neighborNode = constructNode(neighbor, end, costToNeighbor);
                neighborNode.parent = currentNode;

                if (existingNeighbor == null) {
                    openSet.add(neighborNode);
                } else {
                    existingNeighbor.gCost = costToNeighbor;
                    existingNeighbor.fCost = existingNeighbor.gCost + existingNeighbor.hCost;
                    existingNeighbor.parent = currentNode;
                }
            }
        }
    }

    private Node findNodeInOpenSet(Node searchNode) {
        for (Node node : openSet) {
            if (node.equals(searchNode)) {
                return node;
            }
        }
        return null;
    }

    private double calculateDistance(Node from, Node to) {
        int dx = to.x - from.x;
        int dy = to.y - from.y;
        return Math.sqrt(dx * dx + dy * dy);
    }

    private Node constructNode(Node position, Node end, int gCost) {
        Node node = new Node(position.x, position.y);
        node.gCost = gCost;
        node.hCost = calculateHeuristic(node, end);
        node.fCost = node.gCost + node.hCost;
        return node;
    }

    private int calculateHeuristic(Node from, Node target) {
        int dx = Math.abs(target.x - from.x);
        int dy = Math.abs(target.y - from.y);
        return (int) Math.sqrt(dx * dx + dy * dy);
    }

    private boolean nodeIsTraversable(Node point) {
        return !(point.x >= BLACKLIST_MIN_X && point.x <= BLACKLIST_MAX_X && point.y >= BLACKLIST_MIN_Y && point.y <= BLACKLIST_MAX_Y);
    }

    private List<Node> reconstructPath(Node endNode) {
        List<Node> path = new ArrayList<>();
        Node current = endNode;

        while (current != null) {
            path.add(current);
            current = current.parent;
        }

        Collections.reverse(path);
        return path;
    }

    public List<Node> simplifyPath(List<Node> path) {
        if (path.size() <= 2) {
            return new ArrayList<>(path);
        }

        List<Node> simplified = new ArrayList<>();
        simplified.add(path.get(0));

        for (int i = 1; i < path.size() - 1; i++) {
            Node prev = path.get(i - 1);
            Node current = path.get(i);
            Node next = path.get(i + 1);

            if (!isOnStraightLine(prev, current, next)) {
                simplified.add(current);
            }
        }

        simplified.add(path.get(path.size() - 1));
        return simplified;
    }

    private boolean isOnStraightLine(Node p1, Node p2, Node p3) {
        int crossProduct = (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
        return Math.abs(crossProduct) < 2;
    }
}