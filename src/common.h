#ifndef __COMMON_H__
#define __COMMON_H__

#define TEST_

#ifdef TEST_
#	define PRINT_INFO(...) \
		std::printf("[" __FUNCTION__ "]: " __VA_ARGS__)
#else
#	define PRINT_INFO(...) \

#endif

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

typedef struct Position {

    float x, y, z;

    struct Position& operator=(Position p) {
        x = p.x;
        y = p.y;
        z = p.z;
        return *this;
    }

} Position;

typedef struct Color {

    float b, g, r, a;

    struct Color& operator=(Color c) {
        r = c.r;
        g = c.g;
        b = c.b;
        a = c.a;
        return *this;
    }

} Color;

typedef struct CloudPoint {

    Position position;
    Color    color;

    struct CloudPoint& operator=(CloudPoint v) {
        position = v.position;
        color = v.color;
        return *this;
    }

} CloudPoint;

typedef struct Cloud {

    CloudPoint* points;
    size_t size;

    Cloud() : points(nullptr), size(0) {}
    ~Cloud() {
        if (points != nullptr) {
            delete []points;
        }
    }

} Cloud;

#define IMG_WIDTH 1920
#define IMG_HEIGHT 1080
#define WIN_WIDTH 800
#define WIN_HEIGHT 600

#endif // __COMMON_H__
