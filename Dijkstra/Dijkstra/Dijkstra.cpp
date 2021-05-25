// Dijkstra.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <vector>
#include <map>
#include <set>

class reverseMap {
public:
    reverseMap() {}
    ~reverseMap() {}
    int getValueByNode(int node);
    void setValueByNode(int node, int newValue);
    /**
     * @brief Remove the minimum value element and return it
     * @return std::pair<node,value>
    */
    std::pair<int, int> removeMinimumValueElement();
    void insert(int node, int value);
    bool empty();
    bool isIn(int node);
    std::map<int, int> nodeIndexedStore;
    std::multimap<int, int> valueIndexedStore;
private:
    //std::map<int, int> nodeIndexedStore;
    //std::multimap<int, int> valueIndexedStore;
    void changeElementIndex(const int oldIndex, const int newIndex, const int element);
    void safeErase(int node, int value);
};

int reverseMap::getValueByNode(int node) {
    return nodeIndexedStore[node];
}
void reverseMap::setValueByNode(int node, int newValue) {
    int oldValue = nodeIndexedStore[node];
    nodeIndexedStore[node] = newValue;
    changeElementIndex(oldValue, newValue, node);
}

void reverseMap::changeElementIndex(const int oldIndex, const int newIndex, const int element) {
    std::vector<int> findedElements;
    auto it = valueIndexedStore.find(oldIndex);
    while (it != valueIndexedStore.end() && it->first == oldIndex) {
        findedElements.push_back(it->second);
        it++;
    }
    valueIndexedStore.erase(oldIndex);
    for (size_t i = 0; i < findedElements.size(); i++) {
        if (findedElements[i] != element) {
            valueIndexedStore.insert(std::pair<int, int>(oldIndex, findedElements[i]));
        } else {
            valueIndexedStore.insert(std::pair<int, int>(newIndex, findedElements[i]));
        }
    }
}

void reverseMap::safeErase(int node, int value) {
    std::vector<int> findedElements;
    auto it = valueIndexedStore.find(value);
    while (it != valueIndexedStore.end() && it->first == value) {
        findedElements.push_back(it->second);
        it++;
    }
    valueIndexedStore.erase(value);
    for (size_t i = 0; i < findedElements.size(); i++) {
        if (findedElements[i] != node) valueIndexedStore.insert(std::pair<int, int>(value, findedElements[i]));
    }
}

/**
 * @brief Remove the minimum value element and return it
 * @return std::pair<node,value>
*/
std::pair<int, int> reverseMap::removeMinimumValueElement() {
    const int minimumNode = valueIndexedStore.begin()->second;
    const int minimumValue = valueIndexedStore.begin()->first;
    safeErase(minimumNode, minimumValue);
    nodeIndexedStore.erase(minimumNode);
    return std::pair<int, int>(minimumNode, minimumValue);
}

void reverseMap::insert(int node, int value) {
    nodeIndexedStore.insert(std::pair<int, int>(node, value));
    valueIndexedStore.insert(std::pair<int, int>(value, node));
}

bool reverseMap::empty() {
    return nodeIndexedStore.empty() && valueIndexedStore.empty();
}

bool reverseMap::isIn(int node) {
    return nodeIndexedStore.find(node) != nodeIndexedStore.end();
}




class Dijkstra {
public:
    Dijkstra(int nodesCount, int startNode) : nodesCount(nodesCount), startNode(startNode) {
        init();
    }
    ~Dijkstra() {}
    void addEdge(int fromNode, int toNode, int weight);
    void addDoubleEdge(int node1, int node2, int weight);
    std::vector<std::vector<int>> getMatrix();
    void run();
    std::vector<int> from;
    std::vector<std::vector<int>> matrix;
private:
    const int infinite = INT_MAX-1;
    const int empty = -1;
    int nodesCount;
    int startNode;
    //std::vector<std::vector<int>> matrix;
    std::set<int> ready;
    std::vector<int> distance;
    reverseMap next;
    //std::vector<int> from;

    void updateDistance(int node);
    void initNext(int startNode);
    void initFrom(int startNode);
    int getNextValue(int node);
    void updateFrom(int node);
    bool isReady(int node);
    void init();
    void prep();
};

void Dijkstra::addEdge(int fromNode, int toNode, int weight) {
    matrix[fromNode][toNode] = weight;
}

void Dijkstra::addDoubleEdge(int node1, int node2, int weight) {
    matrix[node1][node2] = weight;
    matrix[node2][node1] = weight;
}

void Dijkstra::init() {
    for (size_t i = 0; i < nodesCount; i++) {
        distance.push_back(infinite);
        from.push_back(empty);
        std::vector<int> row;
        for (size_t j = 0; j < nodesCount; j++) {
            row.push_back(empty);
        }
        matrix.push_back(row);
    }
    ready.insert(startNode);
    distance[startNode] = 0;
    initNext(startNode);
    initFrom(startNode);
    from[startNode] = startNode;
}

void Dijkstra::prep() {
    ready.insert(startNode);
    distance[startNode] = 0;
    initNext(startNode);
    initFrom(startNode);
    from[startNode] = startNode;
}

void Dijkstra::initNext(int startNode) {
    for (size_t node = 0; node < nodesCount; node++) {
        if (matrix[startNode][node] != empty) {
            next.insert(node, matrix[startNode][node]);
        }
    }
}

void Dijkstra::initFrom(int startNode) {
    for (size_t i = 0; i < nodesCount; i++) {
        if(matrix[startNode][i] != empty) from[i] = startNode;
    }
}

bool Dijkstra::isReady(int node) {
    return ready.find(node) != ready.end();
}

void Dijkstra::run() {
    prep();
    while (!next.empty()) {
        const std::pair<int, int> minNext = next.removeMinimumValueElement();
        const int minNextNode = minNext.first;
        const int minNextValue = minNext.second;
        ready.insert(minNextNode);
        distance[minNextNode] = minNextValue;
        for (size_t i = 0; i < nodesCount; i++) {
            if (matrix[minNextNode][i] != empty && !isReady(i)) {
                const int newDistance = distance[minNextNode] + matrix[minNextNode][i];
                if (!next.isIn(i)) {
                    next.insert(i, newDistance);
                    from[i]=minNextNode;
                }else{
                    if (next.getValueByNode(i) > newDistance) {
                        next.setValueByNode(i, newDistance);
                        from[i] = minNextNode;
                    }
                }
            }
        }
    }
}

int main() {
    
    Dijkstra dijkstra(5, 0);

    //http://www.cs.bme.hu/~csima/algraf19/eloadas_12.pdf
    /*      0 1 2 3 4 */
    /*      A B C D E */
    /* 0 A| 0 3 0 5 0 */ dijkstra.addEdge(0,1,3); dijkstra.addEdge(0,3,5);
    /* 1 B| 0 0 5 0 0 */ dijkstra.addEdge(1,2,5);
    /* 2 C| 0 0 0 0 3 */ dijkstra.addEdge(2,4,3);
    /* 3 D| 0 1 2 0 1 */ dijkstra.addEdge(3,1,1); dijkstra.addEdge(3,2,2); dijkstra.addEdge(3,4,1);
    /* 4 E| 0 0 2 0 0 */ dijkstra.addEdge(4,2,2);

    dijkstra.run();
    for (size_t i = 0; i < dijkstra.from.size(); i++) {
        std::cout << i << " ";
    }
    std::cout << std::endl;
    for (size_t i = 0; i < dijkstra.from.size(); i++) {
        std::cout << dijkstra.from[i] << " ";
    }
    std::cout << std::endl;





    std::cout << std::endl;
    std::cout << std::endl;

    Dijkstra dijkstra2(10, 0);

    //http://graphonline.ru/en/?graph=YXanHLCRUNHIINeV
    dijkstra2.matrix[0] = std::vector<int>{ -1, 5, -1, -1, -1, -1, -1, -1, -1, -1 };
    dijkstra2.matrix[1] = std::vector<int>{ 5, -1, 7, -1, -1, 1, -1, -1, 42, -1 };
    dijkstra2.matrix[2] = std::vector<int>{ -1, 7, -1, 7, -1, -1, -1, -1, -1, -1 };
    dijkstra2.matrix[3] = std::vector<int>{ -1, -1, -1, -1, -1, -1, 7, -1, -1, -1 };
    dijkstra2.matrix[4] = std::vector<int>{ -1, -1, -1, -1, -1, 7, 7, 7, 1, 7 };
    dijkstra2.matrix[5] = std::vector<int>{ -1, 1, -1, -1, 7, -1, -1, -1, -1, -1 };
    dijkstra2.matrix[6] = std::vector<int>{ -1, -1, -1, -1, 7, -1, -1, 42, -1, -1 };
    dijkstra2.matrix[7] = std::vector<int>{ -1, -1, -1, -1, 7, -1, -1, -1, -1, -1 };
    dijkstra2.matrix[8] = std::vector<int>{ -1, 42, -1, -1, 1, -1, -1, -1, -1, 11 };
    dijkstra2.matrix[9] = std::vector<int>{ -1, -1, -1, -1, 7, -1, -1, -1, 11, -1 };

    dijkstra2.run();
    for (size_t i = 0; i < dijkstra2.from.size(); i++) {
        std::cout << i << " ";
    }
    std::cout << std::endl;
    for (size_t i = 0; i < dijkstra2.from.size(); i++) {
        std::cout << dijkstra2.from[i] << " ";
    }
    std::cout << std::endl;






    std::cout << std::endl;
    std::cout << std::endl;

    Dijkstra dijkstra3(10, 7);

    //http://graphonline.ru/en/?graph=YXanHLCRUNHIINeV
    dijkstra3.matrix[0] = std::vector<int>{ -1, 5, -1, -1, -1, -1, -1, -1, -1, -1 };
    dijkstra3.matrix[1] = std::vector<int>{ 5, -1, 7, -1, -1, 1, -1, -1, 42, -1 };
    dijkstra3.matrix[2] = std::vector<int>{ -1, 7, -1, 7, -1, -1, -1, -1, -1, -1 };
    dijkstra3.matrix[3] = std::vector<int>{ -1, -1, -1, -1, -1, -1, 7, -1, -1, -1 };
    dijkstra3.matrix[4] = std::vector<int>{ -1, -1, -1, -1, -1, 7, 7, 7, 1, 7 };
    dijkstra3.matrix[5] = std::vector<int>{ -1, 1, -1, -1, 7, -1, -1, -1, -1, -1 };
    dijkstra3.matrix[6] = std::vector<int>{ -1, -1, -1, -1, 7, -1, -1, 42, -1, -1 };
    dijkstra3.matrix[7] = std::vector<int>{ -1, -1, -1, -1, 7, -1, -1, -1, -1, -1 };
    dijkstra3.matrix[8] = std::vector<int>{ -1, 42, -1, -1, 1, -1, -1, -1, -1, 11 };
    dijkstra3.matrix[9] = std::vector<int>{ -1, -1, -1, -1, 7, -1, -1, -1, 11, -1 };

    dijkstra3.run();
    for (size_t i = 0; i < dijkstra3.from.size(); i++) {
        std::cout << i << " ";
    }
    std::cout << std::endl;
    for (size_t i = 0; i < dijkstra3.from.size(); i++) {
        std::cout << dijkstra3.from[i] << " ";
    }
    std::cout << std::endl;

    return 0;
}