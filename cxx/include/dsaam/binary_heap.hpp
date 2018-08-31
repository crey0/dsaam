#ifndef DSAAM_BINARY_HEAP_HPP
#define DSAAM_BINARY_HEAP_HPP
#include<vector>
#include<list>
#include<cstddef>
#include<functional>

namespace dsaam
{
  template<typename T, typename C = std::less<T>>
  class binary_heap
  {
  public:

    struct node_type;
    typedef struct node_type
    {
      node_type(size_t index, const T &value) : index(index), value(value) {}
      node_type(size_t index, T &&value) : index(index), value(std::move(value)) {}
      size_t index;
      typename std::list<struct node_type>::const_iterator it_erase;
      T value;
    }node_type;
      
    typedef node_type* handle_type;
    static constexpr handle_type empty_handle = nullptr;
    
    binary_heap() {}
    
    handle_type push(const T& v)
    {
      heap_data_.emplace_front(heap_.size(), v);
      return _update_after_push();
    }

    handle_type push(T&& v)
    {
      heap_data_.emplace_front(heap_.size(), std::move(v));
      return _update_after_push();
    }

    void pop()
    {
      auto hf = heap_.front();
      auto hb = heap_.back();
      swap_nodes(hf, hb);
      heap_.pop_back();
      siftdown(hb);
      heap_data_.erase(hf->it_erase);
    }
      
    T& top() const
    {
      return heap_.front()->value;
    }

    handle_type father(handle_type h)
    {
      return heap_.at((h->index - 1) / 2);
    }

    handle_type left(handle_type h)
    {
      return heap_.at(2 * h->index + 1);
    }

    handle_type right(handle_type h)
    {
      return heap_.at(2 * h->index + 2);
    }

    size_t child_count(handle_type h)
    {
      if(2 * h->index + 1 < size_) 
	return std::min<size_t>(size_ - 2 * h->index - 1, 2);
      else
	return 0;
    }
    
    handle_type best_child(handle_type h)
    {
      auto n = child_count(h);
      switch (n)
	{
	case 0: return empty_handle;
	case 1: return left(h);
	default: 
	  auto l = left(h);
	  auto r = right(h);
	  return compare(l->value, r->value) ? r : l;
	}
    }
      
    void swap_nodes(handle_type a, handle_type b)
    {
      heap_.at(b->index) = a;
      heap_.at(a->index) = b;
      size_t a_idx = a->index;
      a->index = b->index;
      b->index = a_idx;
    }
    
    void update(handle_type h)
    {
      if(h->index != 0 && compare(father(h)->value, h->value))
	{
	  siftup(h);
	}
      else
	{
	  siftdown(h);
	}
    }

    void siftup(handle_type h)
    {
      while(h->index != 0 && compare(father(h)->value, h->value))
	{
	  swap_nodes(h, father(h));
	}
    }

    void siftdown(handle_type h)
    {
      auto c = best_child(h);
      while(c && compare(h->value, c->value))
	{
	  swap_nodes(h,c);
	  c = best_child(h);
	}
    }
    
  private:
    handle_type _update_after_push()
    {
      handle_type h = &heap_data_.front();
      heap_.emplace_back(h);
      h->it_erase = heap_data_.begin();
      size_ = heap_.size();
      siftup(h);
      return h;
    }

  private:
    C compare;
    std::list<node_type> heap_data_;
    std::vector<handle_type> heap_;
    size_t size_;
    
  };
}
#endif
