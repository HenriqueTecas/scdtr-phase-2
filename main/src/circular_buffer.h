#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

struct buffer_data
{
  float lux_value;
  float duty_cycle;
};

template <int CAPACITY = 10>
class CircularBuffer
{
public:
  CircularBuffer() : write_idx(0), count(0) {}

  int capacity() const { return CAPACITY; }
  int size() const { return count; }
  bool empty() const { return count == 0; }
  bool full() const { return count == CAPACITY; }

  void push(buffer_data sample)
  {
    data[write_idx] = sample;
    write_idx = (write_idx + 1) % CAPACITY;
    if (count < CAPACITY)
      count++;
  }

  buffer_data pop()
  {
    if (empty())
      return {0.0f, 0.0f};
    int oldest = (write_idx - count + CAPACITY) % CAPACITY;
    count--;
    return data[oldest];
  }

private:
  buffer_data data[CAPACITY];
  int write_idx;
  int count;
};

#endif