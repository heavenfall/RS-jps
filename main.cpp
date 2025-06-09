#include "main.h"

using namespace std;

static const string mapfile = "den011d.map";
// static const string mapfile = "a.map";
void scanTest()
{
    warthog::domain::gridmap map(mapfile.c_str());    
    jps::jump::jump_point_online jps(&map);
    Scanner scanner(&jps);
    int a, b, steps, xx, yy;
    char c, d;
    bool top; 
    pad_id testid;
    while (true)
    {
        cout<<"x: ";
        cin>>xx;
        cout<<"y: ";
        cin>>yy;
        cout<<"dir: ";
        cin>>c;
        cout<<"top: t   bottom: b:";
        cin>>d;
        switch (d)
        {
        case 't':
            top = true;
            break;
        case 'b':
            top = false;
            break;
        default:
            cout<<"invalid\n";
            continue;
            break;
        }
        //1 is north
        testid = map.to_padded_id_from_unpadded(xx, yy);

        steps = scanner.test_scan(testid, top, c);
        // steps = scanner.scan_west(testid, boo, dir, a, b);
        cout<<"steps: "<<steps<<'\n';
    }
}

void scanTest2()
{
    warthog::domain::gridmap map(mapfile.c_str());    
    jps::jump::jump_point_online jps(&map);
    Scanner scanner(&jps);
    pad_id testid = map.to_padded_id_from_unpadded(153, 61);
    std::vector<pad_id> testvec;
    testvec = scanner.scan(testid, 100, 100);
    std::cout<<"size: "<<testvec.size()<<'\n';
    for(auto iter : testvec)
    {
        uint32_t x, y; 
        map.to_unpadded_xy(iter, x, y);
        std::cout<<"x: "<<x<<" y: "<<y<<'\n';
    }
}

int main(int argc, char const *argv[])
{
    scanTest2();
    return 0;
}
