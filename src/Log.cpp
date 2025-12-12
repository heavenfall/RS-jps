#include <Log.h>

void Tracer::expand(uint32_t x, uint32_t y, std::string color, std::string type)
{
    expand(point((int16_t)x, (int16_t)y), color, type);
}

void Tracer::expand(point p, std::string color, std::string type)
{
    auto x = uint32_t{p.x}, y = uint32_t{p.y};  
    if (adj_for_padding)
    {
        y-=3;
    }
    std::string trace =
    "- { type: " + type + ", tag: grid" + ", color: " + color +
    ", id: " + std::to_string(x) + ":" + std::to_string(y) + 
    ", x: " + std::to_string(x) + ", y: " + std::to_string(y) + "}\n";
    m_trace << trace;
}
void Tracer::draw_cell(point p, std::string color, std::string type)
{
    auto x = uint32_t{p.x}, y = uint32_t{p.y};  
    if (adj_for_padding)
    {
        y-=3;
    }
    std::string trace =
    "- { type: " + type + ", tag: tempGrid" + ", color: " + color +
    ", id: " + std::to_string(x) + ":" + std::to_string(y) + 
    ", x: " + std::to_string(x) + ", y: " + std::to_string(y) + "}\n";
    m_trace << trace;
}

void Tracer::close_node(point p)
{
    auto x = uint32_t{p.x}, y = uint32_t{p.y};  
    if (adj_for_padding)
    {
        y-=3;
    }
    std::string trace =
    "- { type: close, tag: grid, color: green"
    ", id: " + std::to_string(x) + ":" + std::to_string(y) + 
    ", x: " + std::to_string(x) + ", y: " + std::to_string(y) + "}\n";
    m_trace << trace;
}

void Tracer::trace_ray(point start, point finish, std::string color, std::string type)
{
    if (adj_for_padding)
    {
        start.y-=3;
        finish.y-=3;
    }
    std::string s1 = std::to_string((int)finish.x - (int)start.x), s2 = std::to_string((int)finish.y - (int)start.y);
    std::string trace = 
    "- { type: " + type + ", tag: ray" + ", color: "+ color + 
    ", x: " + std::to_string(start.x) + ", y: " + std::to_string(start.y) + 
    ", rayShootX: " +s1 + ", rayShootY: "+s2 + "}\n";
    m_trace << trace;
}

void Tracer::draw_bounds(point p, direction_id dir)
{
    auto xend = point(p.x, adj[(int)dir-4].y == (int16_t)-1 ? 0 : m_dim.height);
    auto yend = point(adj[(int)dir-4].x == (int16_t)-1 ? 0 : m_dim.width, p.y);
    trace_ray_till_close(p, p, xend, "red", "x boundary");
    trace_ray_till_close(p, p, yend, "red", "y boundary");
}

void Tracer::trace_ray_till_close(point closeid, point start, point finish, std::string color, std::string type)
{
    if (adj_for_padding)
    {
        closeid.y-=3;
        start.y-=3;
        finish.y-=3;
    }
    std::string s1 = std::to_string((int)finish.x - (int)start.x), s2 = std::to_string((int)finish.y - (int)start.y);
    std::string trace = 
    "- { type: " + type + ", tag: tempRay" + ", color: "+ color + 
    ", id: " + std::to_string(closeid.x) + ":" + std::to_string(closeid.y) + 
    ", x: " + std::to_string(start.x) + ", y: " + std::to_string(start.y) + 
    ", rayShootX: " +s1 + ", rayShootY: "+s2 + "}\n";
    m_trace << trace;
}

void Tracer::init(point start, point finish)
{
    m_trace.clear();
    auto sy = uint32_t{start.y}, fy = uint32_t{finish.y};    
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
    "    - $: rect\n"
    "      fill: ${{ $.color }}\n"
    "      width: 1\n"
    "      height: 1\n"
    "      x: ${{ $.x }}\n"
    "      y: ${{ $.y }}\n"
    "      clear: true\n"
    "      $if: ${{ $.tag == 'tempGrid'}}\n"
    "    - $: ray\n"
    "      x1: ${{$.x}}\n"
    "      y1: ${{$.y}}\n"
    "      x2: ${{$.x + $.rayShootX}}\n"
    "      y2: ${{$.y + $.rayShootY}}\n"
    "      color: ${{ $.color }}\n"
    "      $if: ${{ $.tag == 'ray'|| $.tag == 'tempRay'}}\n"
    "events:\n";
    m_trace << header;
    std::string type;
    type = "source";
    m_trace << "- { type: " << type << ", tag: grid"<< ", color: green" << ", id: " << std::to_string(start.x) + ":" + std::to_string(start.y) << ", x: " << start.x << ", y: " << sy << "}\n";
    type = "destination";
    m_trace << "- { type: " << type << ", tag: grid"<< ", color: blue" <<", id: " << std::to_string(finish.x) + ":" + std::to_string(finish.y) << ", x: " << finish.x << ", y: " << fy << "}\n";
}

void Tracer::close()
{
    m_trace.close();
}
