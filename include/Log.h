#pragma once
#include <string>
#include <fstream>
#include <fstream>
#include <iostream>
#include <array>
#include <jps/forward.h>
#include <memory>
#include <warthog/domain/gridmap.h>

using namespace std;

class Tracer
{
private:
    const string m_fileName;
    ofstream m_trace;
    const bool adj_for_padding = true;
public:
    Tracer(string _fname) : m_fileName(_fname), m_trace(_fname){};
    Tracer() : m_trace(ofstream{"myTrace.trace.yaml"}) {};
    ~Tracer() {};

    void init(pair<uint32_t, uint32_t> start, pair<uint32_t, uint32_t> finish);
    void expand(uint32_t x, uint32_t y);
    void expand(std::pair<uint32_t, uint32_t> p);
    void expand(std::pair<uint32_t, uint32_t> p, string color, string tag);
    void trace_ray(pair<uint32_t, uint32_t> start, pair<uint32_t, uint32_t> finish);
    void trace_ray(pair<uint32_t, uint32_t> start, pair<uint32_t, uint32_t> finish, string color, string tag);
};
