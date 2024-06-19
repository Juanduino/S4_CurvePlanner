#include "CircularBuffer.h"

float mm_per_rev = 17.357f;

CircularBuffer::CircularBuffer(int size) : size(size), head(0), tail(0) {
    buffer = new char*[size];
}

void CircularBuffer::push(char* item) {
    buffer[head] = item;
    head = (head + 1) % size;
}

void CircularBuffer::pop(char* &tailItem, char* &nextItem) {
    if (isEmpty()) {
        tailItem = nullptr;
        nextItem = nullptr;
        return;
    }
    
    tailItem = buffer[tail];

    // Calculate the next tail position
    int nextTail = (tail + 1) % size;
    
    // Check if it's the last command in the buffer
    if (nextTail == head) {
        nextItem = nullptr; // Set nextItem to nullptr or 0
    } else {
        nextItem = buffer[nextTail];
    }

    // Move the tail to the next position in the circular buffer
    tail = nextTail;
}

bool CircularBuffer::isEmpty() {
    return head == tail;
}