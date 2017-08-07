#ifndef DSAAM_QUEUE_HPP
#define DSAAM_QUEUE_HPP


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
	std::lock_guard(m);
	tokens +=1;
      }
      cv.notify_one();
    }

    void decrease()
    {
      std::unique_lock lk(m);
      cv.wait(m, []{return bounded > 0});
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
	: max_size(max_size), head(0), tail(0), n_full_to_pop(0), n_free_to_push(max_size)
      {
	buffer = new M[max_size];
      }

      void push(M && m)
      {
	n_full_to_push.decrease();//blocking
	buffer[tail] = std::move(m);
	tail = (tail + 1) % max_size;
	n_free_to_pop.increase();
      }

      M&& pop()
      {
	n_full_to_pop.decrease();//blocking
	M && m = std::move(buffer[head]);
	head = (head - 1) % max_size;
	n_free_to_push.increase();
	return M
      }
    private:
      unsigned int max_size;
      unsigned int head;
      unsigned int tail;
      Semaphore n_full_to_pop;
      Semaphore n_free_to_push;
      M buffer[];
    };
    
    }
#endif //DSAAM_QUEUE_HPP
