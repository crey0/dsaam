#ifndef DSAAM_QUEUE_HPP
#define DSAAM_QUEUE_HPP
#include <mutex>
#include <condition_variable>
#include <vector>

namespace dsaam
{

  class Semaphore
  {
  public:
    Semaphore(unsigned int tokens, bool bounded=false)
      : tokens(tokens), bounded(bounded) {}

    void increase()
    {
      {
	std::lock_guard<std::mutex>(this->m);
	tokens +=1;
      }
      cv.notify_one();
    }

    void decrease()
    {
      std::unique_lock<std::mutex> lk(m);
      cv.wait(lk, [this]{return tokens > 0;});
      tokens -=1;
      lk.unlock();
      cv.notify_one();
    }
    
  private:
    unsigned int tokens;
    bool bounded;
    std::mutex m;
    std::condition_variable cv;
    
  };
  
  template<typename M>
  class Queue
  {
  public:
    Queue(unsigned int max_size)
      : max_size(max_size), head(0), tail(0), n_full_to_pop(0), n_free_to_push(max_size),
	buffer(max_size) {}

    ~Queue() {}

    void push(M &&e)
    {
      n_free_to_push.decrease();//blocking
      buffer[tail] = std::move(e);
      tail = (tail + 1) % max_size;
      n_full_to_pop.increase();
    }

    M&& pop()
    {
      n_full_to_pop.decrease();//blocking
      M && e = std::move(buffer[head]);
      head = (head - 1) % max_size;
      n_free_to_push.increase();
      return std::move(e);
    }
  private:
    unsigned int max_size;
    unsigned int head;
    unsigned int tail;
    Semaphore n_full_to_pop;
    Semaphore n_free_to_push;
    std::vector<M> buffer;
  };
    
}
#endif //DSAAM_QUEUE_HPP
