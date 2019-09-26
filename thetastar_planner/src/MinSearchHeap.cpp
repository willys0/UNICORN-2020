#include "thetastar_planner/MinSearchHeap.h"

MinSearchHeap::MinSearchHeap() {
	element_ = 0; //nullptr
	buffer = 0; //nullptr
	heap_size = 0;
	buffer_size = 0;
}

MinSearchHeap::MinSearchHeap(int heapsize) {
	if (heapsize <= 0) return;
	element_ = new HeapElement[heapsize];
	buffer = new int[heapsize];
	buffer_size = heapsize;
	heap_size = 0;
}

MinSearchHeap::~MinSearchHeap() {
	if (buffer_size) {
		delete element_;
		delete buffer;
		element_ = 0; //nullptr
		buffer = 0; //nullptr
		buffer_size = 0;
		heap_size = 0;
	}
}

bool MinSearchHeap::Insert(HeapElement x) {
	int i, parent;
	if (heap_size == buffer_size) return false;
	i = heap_size;
	heap_size++;
	parent = (i >> 1);
	while (i > 0 && element_[parent].key > x.key) {
		element_[i] = element_[parent];
		i = parent;
		parent = (i >> 1);
	}
	element_[i] = x;
	return true;
}

void MinSearchHeap::MinHeapify() {
	int l, r, least, i;
	HeapElement tmp;
	int buffersize = 0;
	buffer[buffersize] = 0; 
	buffersize++;
	while (buffersize) {
		buffersize--;
		i = buffer[buffersize];
		l = (i << 1);
		r = ((i << 1) + 1);
		if (l < heap_size && element_[l].key < element_[i].key)
			least = l;
		else
			least = i;
		if (r < heap_size && element_[r].key < element_[least].key)
			least = r;
		if (least != i) {
			tmp = element_[i];
			element_[i] = element_[least];
			element_[least] = tmp;
			buffer[buffersize] = least;
			buffersize++;
		}
	}
}

HeapElement MinSearchHeap::SearchMin() {
	HeapElement min_node;
	min_node = element_[0];
	element_[0] = element_[heap_size - 1];
	heap_size--;
	MinHeapify();
	return min_node;
}

bool MinSearchHeap::IsEmpty() {
	if (heap_size)
		return false;
	else
		return true;
}

void MinSearchHeap::Clear() {
	heap_size = 0;
}