#include <Log.h>

void Tracer::expand(uint32_t x, uint32_t y)
{
    if (adj_for_padding)
    {
        y-=3;
    }
    m_trace << "- { type: " << "expanding" << ", color: yellow" << ", id: " << to_string(x) + ":" + to_string(y) << ", x: " << to_string(x) << ", y: " << to_string(y) << "}\n";
}

void Tracer::expand(std::pair<uint32_t, uint32_t> p)
{
    auto x = uint32_t{p.first}, y = uint32_t{p.second};  
    if (adj_for_padding)
    {
        y-=3;
    }
    m_trace << "- { type: " << "expanding" << ", color: yellow" << ", id: " << to_string(x) + ":" + to_string(y) << ", x: " << to_string(x) << ", y: " << to_string(y) << "}\n";
}

void Tracer::expand(std::pair<uint32_t, uint32_t> p, string color, string tag)
{
    auto x = uint32_t{p.first}, y = uint32_t{p.second};  
    if (adj_for_padding)
    {
        y-=3;
    }
    m_trace << "- { type: " << tag << ", color: " << color << ", id: " << to_string(x) + ":" + to_string(y) << ", x: " << to_string(x) << ", y: " << to_string(y) << "}\n";
}

void Tracer::trace_ray(pair<uint32_t, uint32_t> start, pair<uint32_t, uint32_t> finish)
{
    if (adj_for_padding)
    {
        start.second-=3;
        finish.second-=3;
    }
    string s1 = to_string((int)finish.first - (int)start.first), s2 = to_string((int)finish.second - (int)start.second);
    m_trace << "- { type: " << "ray shooting" << ", color: green" << ", x: " << to_string(start.first) << ", y: " << to_string(start.second) << ", rayShootX: " <<s1 << ", rayShootY: "<<s2 << "}\n";
}

void Tracer::trace_ray(pair<uint32_t, uint32_t> start, pair<uint32_t, uint32_t> finish, string color, string tag)
{
    if (adj_for_padding)
    {
        start.second-=3;
        finish.second-=3;
    }
    string s1 = to_string((int)finish.first - (int)start.first), s2 = to_string((int)finish.second - (int)start.second);
    m_trace << "- { type: " << tag << ", color: "<< color << ", x: " << to_string(start.first) << ", y: " << to_string(start.second) << ", rayShootX: " <<s1 << ", rayShootY: "<<s2 << "}\n";
}

void Tracer::init(pair<uint32_t, uint32_t> start, pair<uint32_t, uint32_t> finish)
{
    m_trace.clear();
    auto sy = uint32_t{start.second}, fy = uint32_t{finish.second};    
    if (adj_for_padding)
    {
        sy-=3; fy-=3;
    }
    m_trace << "version: 1.4.0\n";

    m_trace << "views:\n  main:\n    - $: rect\n      fill: ${{ $.color }}\n      width: 1\n      height: 1\n      x: ${{ $.x }}\n      y: ${{ $.y }}\n";

    m_trace << "    - $: path\n      points:\n        - x: ${{$.x}}\n          y: ${{$.y}}\n        - x: ${{$.x + $.rayShootX}}\n          y: ${{$.y + $.rayShootY}}\n";
    m_trace << "      fill: ${{ $.color }}\n      alpha: 0.5\n      lineWidth: 0.3\n      display: transient\n      clear: true\n";

    m_trace << "events:\n";
    string type;
    type = "source";
    m_trace << "- { type: " << type << ", color: blue" << ", id: " << to_string(start.first) + ":" + to_string(start.second) << ", x: " << start.first << ", y: " << sy << "}\n";
    type = "destination";
    m_trace << "- { type: " << type << ", color: green" << ", id: " << to_string(finish.first) + ":" + to_string(finish.second) << ", x: " << finish.first << ", y: " << fy << "}\n";
}