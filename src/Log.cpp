#include <Log.h>

void Tracer::expand(uint32_t x, uint32_t y, string color, string type)
{
    expand(std::make_pair(x, y), color, type);
}

void Tracer::expand(std::pair<uint32_t, uint32_t> p, string color, string type)
{
    auto x = uint32_t{p.first}, y = uint32_t{p.second};  
    if (adj_for_padding)
    {
        y-=3;
    }
    std::string trace =
    "- { type: " + type + ", tag: grid" + ", color: " + color +
    ", id: " + to_string(x) + ":" + to_string(y) + 
    ", x: " + to_string(x) + ", y: " + to_string(y) + "}\n";
    m_trace << trace;
}

void Tracer::close_node(std::pair<uint32_t, uint32_t> p)
{
    auto x = uint32_t{p.first}, y = uint32_t{p.second};  
    if (adj_for_padding)
    {
        y-=3;
    }
    std::string trace =
    "- { type: close, tag: grid, color: green"
    ", id: " + to_string(x) + ":" + to_string(y) + 
    ", x: " + to_string(x) + ", y: " + to_string(y) + "}\n";
    m_trace << trace;
}

void Tracer::trace_ray(pair<uint32_t, uint32_t> start, pair<uint32_t, uint32_t> finish, string color, string type)
{
    if (adj_for_padding)
    {
        start.second-=3;
        finish.second-=3;
    }
    string s1 = to_string((int)finish.first - (int)start.first), s2 = to_string((int)finish.second - (int)start.second);
    std::string trace = 
    "- { type: " + type + ", tag: ray" + ", color: "+ color + 
    ", x: " + to_string(start.first) + ", y: " + to_string(start.second) + 
    ", rayShootX: " +s1 + ", rayShootY: "+s2 + "}\n";
    m_trace << trace;
}

void Tracer::draw_bounds(std::pair<uint32_t, uint32_t> p, direction dir)
{
    uint32_t dir_ind = std::countr_zero<uint8_t>(dir)-4;
    auto xend = std::make_pair(p.first, adj[dir_ind][1] == -1 ? 0 : m_dim.height);
    auto yend = std::make_pair(adj[dir_ind][0] == -1 ? 0 : m_dim.width, p.second);
    trace_ray_till_close(p, p, xend, "red", "x boundary");
    trace_ray_till_close(p, p, yend, "red", "y boundary");
}

void Tracer::trace_ray_till_close(std::pair<uint32_t, uint32_t> closeid, pair<uint32_t, uint32_t> start, pair<uint32_t, uint32_t> finish, string color, string type)
{
    if (adj_for_padding)
    {
        closeid.second-=3;
        start.second-=3;
        finish.second-=3;
    }
    string s1 = to_string((int)finish.first - (int)start.first), s2 = to_string((int)finish.second - (int)start.second);
    std::string trace = 
    "- { type: " + type + ", tag: tempRay" + ", color: "+ color + 
    ", id: " + to_string(closeid.first) + ":" + to_string(closeid.second) + 
    ", x: " + to_string(start.first) + ", y: " + to_string(start.second) + 
    ", rayShootX: " +s1 + ", rayShootY: "+s2 + "}\n";
    m_trace << trace;
}

void Tracer::init(pair<uint32_t, uint32_t> start, pair<uint32_t, uint32_t> finish)
{
    m_trace.clear();
    auto sy = uint32_t{start.second}, fy = uint32_t{finish.second};    
    if (adj_for_padding)
    {
        sy-=3; fy-=3;
    }
    std::string header = 
    "version: 1.4.0\n"
    "views:\n"
    "  ray:\n"
    "    - $: path\n"
    "      points: \n"
    "        - x: ${{$.x1+0.5}}\n"
    "          y: ${{$.y1+0.5}}\n"
    "        - x: ${{$.x2+0.5}}\n"
    "          y: ${{$.y2+0.5}}\n"
    "      fill: ${{ $.color }}\n"
    "      alpha: 0.5\n"
    "      line-width: >-\n"
    "        ${{ $.type == 'scanning' ? 1 : 0.3 }}\n"
    "      clear: >-\n"
    "        ${{ $.tag == 'tempRay' ? 'close' : true }}\n"
    "  main:\n"
    "    - $: rect\n"
    "      fill: ${{ $.color }}\n"
    "      width: 1\n"
    "      height: 1\n"
    "      x: ${{ $.x }}\n"
    "      y: ${{ $.y }}\n"
    "      $if: ${{ $.tag == 'grid'}}\n"
    "    - $: ray\n"
    "      x1: ${{$.x}}\n"
    "      y1: ${{$.y}}\n"
    "      x2: ${{$.x + $.rayShootX}}\n"
    "      y2: ${{$.y + $.rayShootY}}\n"
    "      color: ${{ $.color }}\n"
    "      $if: ${{ $.tag == 'ray'|| $.tag == 'tempRay'}}\n"
    "events:\n";
    m_trace << header;
    string type;
    type = "source";
    m_trace << "- { type: " << type << ", tag: grid"<< ", color: green" << ", id: " << to_string(start.first) + ":" + to_string(start.second) << ", x: " << start.first << ", y: " << sy << "}\n";
    type = "destination";
    m_trace << "- { type: " << type << ", tag: grid"<< ", color: blue" <<", id: " << to_string(finish.first) + ":" + to_string(finish.second) << ", x: " << finish.first << ", y: " << fy << "}\n";
}

void Tracer::close()
{
    m_trace.close();
}
