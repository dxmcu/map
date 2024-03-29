#ifndef TIC_TOC_H_
#define TIC_TOC_H_

#pragma once

#include <ctime>
#include <cstdlib>
#include <chrono>
#include <iostream>

class TicToc
{
  public:
    TicToc()
    {
        Tic();
    }

    void Tic()
    {
        start = std::chrono::system_clock::now();
    }

    double Toc()
    {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count() * 1000;
    }

  private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};

#endif // TIC_TOC_H_
