#include "main.h"
using namespace std;
static const double EPSILON = 0.000001f;

// void scanTest()
// {
//     warthog::domain::gridmap map(mapname.c_str());    
//     jps::jump::jump_point_online jps(&map);
//     Scanner scanner(std::make_shared<Tracer>(), &jps);
//     int a, b, steps, xx, yy;
//     char c, d;
//     bool top; 
//     pad_id testid;
//     while (true)
//     {
//         cout<<"x: ";
//         cin>>xx;
//         cout<<"y: ";
//         cin>>yy;
//         cout<<"dir: ";
//         cin>>c;
//         cout<<"top: t   bottom: b:";
//         cin>>d;
//         switch (d)
//         {
//         case 't':
//             top = true;
//             break;
//         case 'b':
//             top = false;
//             break;
//         default:
//             cout<<"invalid\n";
//             continue;
//             break;
//         }
//         //1 is north
//         testid = map.to_padded_id_from_unpadded(xx, yy);
//         steps = scanner.test_scan_single(testid, top, c);
//         // steps = scanner.scan_west(testid, boo, dir, a, b);
//         cout<<"steps: "<<steps<<'\n';
//     }
// }

// void scanTest2()
// {
//     warthog::domain::gridmap map(mapname.c_str());    
//     jps::jump::jump_point_online jps(&map);
//     Scanner scanner(std::make_shared<Tracer>(), &jps);
//     pad_id testid = map.to_padded_id_from_unpadded(153, 61);
//     std::vector<pad_id> testvec;
//     testvec = scanner.test_scan_full(testid, 100, 100);
//     std::cout<<"size: "<<testvec.size()<<'\n';
//     for(auto iter : testvec)
//     {
//         uint32_t x, y; 
//         map.to_unpadded_xy(iter, x, y);        
//         std::cout<<"x: "<<x<<" y: "<<y<<'\n';
//     }
// }

template <bool Test>
void run_scenario(warthog::util::scenario_manager &scen_mngr, string mapname)
{
    warthog::domain::gridmap map(mapname.c_str());
    jps::domain::rotate_gridmap rmap(map);
    jps::jump::jump_point_online jps(rmap);
    auto s = Solver<SolverTraits::Default>(&jps, rmap);
    std::cout << "experiment\t\toptimal plenth\t\trjps plenth\t\texpanded\t\tgenerated\t\treopend\t\tupdated\t\theap_pops\t\tnanos\t\texpansion time\n";
    // Solver<2> s(&jps);
    for (size_t i = 0; i < scen_mngr.num_experiments(); i++)
    {
        const auto& cur_exp = scen_mngr.get_experiment(i);
        pad_id start = map.to_padded_id_from_unpadded(uint32_t(cur_exp->startx()), uint32_t(cur_exp->starty()));
        pad_id target = map.to_padded_id_from_unpadded(uint32_t(cur_exp->goalx()), uint32_t(cur_exp->goaly()));
        s.get_path(grid_id(start), grid_id(target));
        auto res = s.get_result();
        // if (std::fabs(res.plenth - cur_exp->distance()) > EPSILON) assert(false &&"failed ");
        if constexpr(Test)
        {
            if (std::fabs(res.plenth - cur_exp->distance()) <= EPSILON) std::cout << "\033[1;32m";  //green 
            else                                                        std::cout << "\033[1;31m";  //red
        }
        std::cout << i;
        std::cout << "\t\t" << std::fixed << std::setprecision(10) << cur_exp->distance();
        std::cout << "\t\t"  << std::fixed << std::setprecision(10) << res.plenth;
        std::cout << "\t\t"  << res.expanded;
        std::cout << "\t\t"  << res.generated;
        std::cout << "\t\t"  << res.reopend;
        std::cout << "\t\t"  << res.updated;
        std::cout << "\t\t"  << res.heap_pops;
        std::cout << "\t\t"  << to_string(res.nanos.count());
        std::cout << "\t\t"  << to_string(res.ray_scan_time/res.nanos.count());
        if constexpr(Test) std::cout << "\033[0m";

        std::cout << '\n';
    }
}

void run_single_test(warthog::util::scenario_manager &scen_mngr, string mapname, size_t i)
{
    warthog::domain::gridmap map(mapname.c_str());
    jps::domain::rotate_gridmap rmap(map);
    jps::jump::jump_point_online jps(rmap);
    auto s = Solver<SolverTraits::OutputToPosthoc>(&jps, rmap);
    const auto& cur_exp = scen_mngr.get_experiment(i);
    pad_id start = map.to_padded_id_from_unpadded(uint32_t(cur_exp->startx()), uint32_t(cur_exp->starty()));
    pad_id target = map.to_padded_id_from_unpadded(uint32_t(cur_exp->goalx()), uint32_t(cur_exp->goaly()));
    s.get_path(grid_id(start), grid_id(target));
    auto res = s.get_result();
    std::cout<< "experiment " << i <<":\t";
    if (std::fabs(res.plenth - cur_exp->distance()) <= EPSILON) std::cout << "\033[1;32m";  //green 
    else                                                        std::cout << "\033[1;31m";  //red

    std::cout << "optimal plenth: " << std::fixed << std::setprecision(10) << cur_exp->distance();
    std::cout << "\trjps plenth: " << std::fixed << std::setprecision(10) <<res.plenth;
    std::cout << "\texpanded: " << res.expanded;
    std::cout << "\tgenerated: " << res.generated;
    std::cout << "\tre-opend: " << res.reopend;
    std::cout << "\tupdated: " << res.updated;
    std::cout << "\theap_pops: " << res.heap_pops;
    std::cout << "\tnanos: " << to_string(res.nanos.count());
    std::cout << "\033[0m\n";
}

static const std::string DEBUG_MAP = "den011d";
constexpr bool DEBUG = false;
static const int DEBUG_CASE = 60;
int main(int argc, char** argv)
{
    // parse arguments
    warthog::util::param valid_args[]
        = {{"alg", required_argument, 0, 1},
        {"scen", required_argument, 0, 0},
        {"map", required_argument, 0, 1},
        // {"gen", required_argument, 0, 3},
        {"help", no_argument, 0, 1},
        {"checkopt", no_argument, 0, 1},
        {"verbose", no_argument, 0, 1},
        {"costs", required_argument, 0, 1},
        {"test", required_argument, 0, 0},
        {0, 0, 0, 0}};

    warthog::util::cfg cfg;
    cfg.parse_args(argc, argv, "a:b:c:def", valid_args);

    std::string sfile = cfg.get_param_value("scen");
    std::string alg   = cfg.get_param_value("alg");
    std::string mapfile  = cfg.get_param_value("map");
    std::string test  = cfg.get_param_value("test");

    auto scen_mngr = warthog::util::scenario_manager{};

    if constexpr (DEBUG)
    {
        std::string DEBUG_SCEN = {"../maps/" + DEBUG_MAP + ".map.scen"};
        std::string DEBUG_MAP_NAME  = {"../maps/" + DEBUG_MAP + ".map"};
        
        scen_mngr.load_scenario(DEBUG_SCEN.c_str());
        run_single_test(scen_mngr, DEBUG_MAP_NAME, DEBUG_CASE);
        return 0;
    }
    scen_mngr.load_scenario(sfile.c_str());

    if(test == "")
    {
        std::puts("please specify --test, y == output to console n == pipe to tsv");
        return 1;
    }
    if(alg == "")
    {
        std::puts("please specify algorithm with --alg");
        return 1;
    }    
    if(sfile == "")
    {
        std::puts("scen or map file empty, specify with --scen or --map");
        return 1;
    }     
    if(alg != "rjps")
    {
        std::puts("only rjps is supported");
        return 1;
    }
    if(mapfile == "")
    {
        // first, try to load the map from the scenario file
        mapfile = warthog::util::find_map_filename(scen_mngr, sfile);
        if(mapfile.empty())
        {
            std::cerr << "could not locate a corresponding map file\n";
            return 3;
        }
    }

    if(test == "y")run_scenario<true>(scen_mngr, mapfile);
    else           run_scenario<false>(scen_mngr, mapfile);
    return 0;
}
