#ifndef CIRCULARBUFFER_H
#define CIRCULARBUFFER_H

class CircularBuffer {
public:
    CircularBuffer(int size);
    void push(char* item);
    void pop(char* &tailItem, char* &nextItem); // Modified pop function
    bool isEmpty();

    //Set the mm per revolution of the drivetrain. This is used to calculate the distance traveled in mm
    float mm_per_rev = 17.3474f;

private:
    char** buffer;
    int head;
    int tail;
    int size;
};

#endif // CIRCULARBUFFER_H