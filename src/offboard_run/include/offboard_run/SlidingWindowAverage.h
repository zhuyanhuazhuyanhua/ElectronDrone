#pragma once
#include <cmath>
#include <queue>

class SlidingWindowAverage {
public:
  SlidingWindowAverage(int windowSize)
      : windowSize(windowSize), windowSum(0.0), windowAvg(0.0) {}

  double addData(double newData) {
    if (!dataQueue.empty() &&
        std::fabs(newData - dataQueue.back()) > resetThreshold) {
      reset(newData);
    } else {
      dataQueue.push(newData);
      windowSum += newData;
    }

    if (dataQueue.size() > windowSize) {
      windowSum -= dataQueue.front();
      dataQueue.pop();
    }

    windowAvg = windowSum / dataQueue.size();
    return windowAvg;
  }

  int getSize() const { return dataQueue.size(); }
  double getAverage() const { return windowAvg; }

private:
  void reset(double val) {
    std::queue<double> empty;
    std::swap(dataQueue, empty);
    windowSum = val;
    dataQueue.push(val);
    windowAvg = val;
  }

  int windowSize;
  double windowSum;
  double windowAvg;
  std::queue<double> dataQueue;
  const double resetThreshold = 0.01;
};
