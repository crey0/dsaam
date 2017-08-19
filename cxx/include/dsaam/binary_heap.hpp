#ifndef BINARY_HEAP_HPP
#define BINARY_HEAP_HPP
#include<vector>
#include<forward_list>

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
      size_t index;
      typename std::forward_list<struct node_type>::const_iterator it_erase;
      T value;
    }node_type;
      
    typedef node_type* handle_type;
    static constexpr handle_type empty_handle = nullptr;
    
    binary_heap() {}
    
    handle_type push(const T& v)
    {
      heap_data_.emplace_front(heap_.size(), v);
      handle_type h = &heap_data_.front();
      heap_.emplace_back(h);
      h->it_erase = heap_data_.before_begin();
      size_ = heap_.size();
      return h;
    }

    void pop()
    {
      swap_nodes(heap_.front(), heap_.back());
      heap_.pop_back();
      decrease(heap_.front());
    }
      
    T& top() const
    {
      return heap_.front()->value;
    }

    handle_type father(handle_type h)
    {
      return heap_[(h->index - 1) / 2];
    }

    handle_type left(handle_type h)
    {
      return heap_[2 * h->index + 1];
    }

    handle_type right(handle_type h)
    {
      return heap_[2 * h->index + 2];
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
	  return compare(r->value, l->value) ? r : l;
	}
    }
      
    void swap_nodes(handle_type a, handle_type b)
    {
      heap_[b->index] = a;
      heap_[a->index] = b;
      size_t a_idx = a->index;
      a->index = b->index;
      b->index = a_idx;
    }
    
    void update(handle_type h)
    {
      if(h->index != 0 && compare(h->value, father(h)->value))
	{
	  increase(h);
	}
      else
	{
	  decrease(h);
	}
    }

    void increase(handle_type h)
    {
      while(h->index != 0 && compare(h->value, father(h)->value))
	{
	  swap_nodes(h, father(h));
	}
    }

    void decrease(handle_type h)
    {
      auto c = best_child(h);
      while(c && compare(c->value, h->value))
	{
	  swap_nodes(h,c);
	  c = best_child(h);
	}
    }
    
  private:
    C compare;
    std::forward_list<node_type> heap_data_;
    std::vector<handle_type> heap_;
    size_t size_;
    
  };
}
#endif
