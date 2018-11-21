/*
Copyright © 2018 CNRS
All rights reserved.

@author Christophe Reymann

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
 2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef DSAAM_COMMON_HPP
#define DSAAM_COMMON_HPP

#include<dsaam/exceptions.hpp>
#include<dsaam/string_utils.hpp>
#include<functional>
#include<memory>
#include<vector>

namespace dsaam
{

  using std::string;

  template<class M, class T, template <class> class F> struct InFlow;
  template<class M, class T, template <class> class F, class S> struct OutFlow;
  struct Sink;

  template<class M, class T, template <class> class F>
  using  message_callback_type =  F<void(const M&, const T &)>;

  template<class M, template <class> class F>
  using send_callback_type = F<void(const M &)>;

  template<class T, template <class> class F>
  using time_callback_type = F<void(const T &)>;

  template<class T>
  struct empty
  {
    static constexpr const T value()
    {
      return T();
    }
  };


  template<class M, class T, template <class> class F>
  struct InFlow
  {
    InFlow() = default;
    InFlow(string name, const T& time, const T& dt, size_t qsize = 0,
	   const message_callback_type<M,T,F> &callback = empty<message_callback_type<M,T,F>>::value(),
	   const time_callback_type<T,F> &time_callback = empty<time_callback_type<T,F>>::value())
      : name(name), time(time), dt(dt), qsize(qsize), callback(callback), time_callback(time_callback) {}
    string name;
    T time;
    T dt;
    size_t qsize;
    message_callback_type<M, T, F> callback;
    time_callback_type<T, F> time_callback;
  };
  
  struct Sink
  {
    Sink() = default;
    Sink(string name)
      : name(name) {}
    string name;
  };


  enum class FlowType { PRED, OBS, _SIZE };


  template<class M, class T, template <class> class F, class S = Sink>
  struct OutFlow
  {
    OutFlow() = default;
    OutFlow(string name, const T& time, const T& dt,
	    const std::vector<S>& sinks = {}, size_t qsize = 0,
	    const send_callback_type<M,F> &send =
	    empty<send_callback_type<M,F>>::value(),
	    FlowType ftype = FlowType::PRED)
      : name(name), time(time), dt(dt), qsize(qsize), send(send), sinks(sinks),
	ftype(ftype) {}

    void setup_sink(S &&sink)
    {
      sinks.emplace_back(std::move(sink));
    }
    
    string name;
    T time;
    T dt;
    size_t qsize;
    send_callback_type<M, F> send;
    std::vector<S> sinks;
    FlowType ftype;
  };

}

#endif
