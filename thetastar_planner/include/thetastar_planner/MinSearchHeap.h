#pragma once
//Implementation of MinHeap to efficiently search for the minimum value from an array
using namespace std;
struct HeapElement {
	float key;
	int data;
};

class MinSearchHeap {
	HeapElement *element_;
	int *buffer;
	int buffer_size;
	int heap_size;
public:
	MinSearchHeap();
	MinSearchHeap(int);
	~MinSearchHeap();
	bool Insert(HeapElement);
	void MinHeapify();
	HeapElement SearchMin();
	bool IsEmpty();
	void Clear();
};