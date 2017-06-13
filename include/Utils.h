//
// Created by glawless on 13.06.17.
//

#ifndef UTILS_H
#define UTILS_H

#include <chrono>
#include <utility>

// Adapted from https://stackoverflow.com/a/33900479
typedef std::chrono::high_resolution_clock::time_point TimeVar;
#define duration(a) std::chrono::duration_cast<std::chrono::nanoseconds>(a).count()
#define timeNow() std::chrono::steady_clock::now()
#define nanoToMilli(t) t*1e-6

template<typename F, typename... Args>
double funcTime(F func, Args&&... args){
    auto t1=timeNow();
    func(std::forward<Args>(args)...);
    return duration(timeNow()-t1);
}

#endif //UTILS_H
