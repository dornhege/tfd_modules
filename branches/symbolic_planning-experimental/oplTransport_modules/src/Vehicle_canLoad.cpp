
#include <deque>
#include "Vehicle_canLoad.h"

namespace opl
{

namespace TransportModules
{

using namespace std;

struct quader
{
    double sx;
    double sy;
    double sz;
    quader(double x, double y, double z) :
        sx(x), sy(y), sz(z)
    {
    }
};

#define DEBUG_MOD (0)

void packItem(deque<double> & items, quader q)
{
    // sort quader
    if (q.sx > q.sy)
    {
        double t = q.sx;
        q.sx = q.sy;
        q.sy = t;
    }
    if (q.sy > q.sz)
    {
        double t = q.sz;
        q.sz = q.sy;
        q.sy = t;
    }
    if (q.sx > q.sy)
    {
        double t = q.sx;
        q.sx = q.sy;
        q.sy = t;
    }

    double biggest = -1;
    for (deque<double>::iterator it = items.begin(); it != items.end(); it++)
    {
        if (*it > q.sx)
            continue;
        biggest = *it;
        items.erase(it);
        break;
    }
    if (biggest == -1)
        return;

    quader smallestQ(q.sx - biggest, biggest, biggest);
    quader mediumQ(q.sx, q.sy - biggest, biggest);
    quader largestQ(q.sx, q.sy, q.sz - biggest);

    packItem(items, smallestQ);
    packItem(items, mediumQ);
    packItem(items, largestQ);
}

bool checkLoading(deque<double> sizes, double cap)
{
    deque<double> itemSizes;
    double third = 1.0 / 3.0;
    quader full(pow(cap, third), pow(cap, third), pow(cap, third));
    for (deque<double>::iterator it = sizes.begin(); it != sizes.end(); it++)
    {
        itemSizes.push_back(pow(*it, third));
    }
    sort(itemSizes.begin(), itemSizes.end());
    reverse(itemSizes.begin(), itemSizes.end());
    packItem(itemSizes, full);
    return itemSizes.empty();
}

//double can_load(const ParameterList & parameterList, predicateCallbackType predicateCallback, numericalFluentCallbackType numericalFluentCallback, int, plannerContextPtr, plannerContextCompareType, bool & tookContext)
//{
//    string me = parameterList.front().value;
//    string pack = parameterList.back().value;
//
//    deque<string> packs_in_truck;
//    for (PredicateList::iterator it = list->begin(); it != list->end(); it++)
//    {
//        Predicate p = *it;
//        if (!p.value)
//            continue;
//        if (p.name != "in")
//            continue;
//        if (p.parameters.back().value == me)
//            packs_in_truck.push_back(p.parameters.front().value);
//    }
//    packs_in_truck.push_back(pack);
//
//    NumericalFluentList* nfl = new NumericalFluentList();
//    for (deque<string>::iterator it = packs_in_truck.begin(); it != packs_in_truck.end(); it++)
//    {
//        ParameterList pl;
//        pl.push_back(Parameter("p", "package", *it));
//        nfl->push_back(NumericalFluent("package-size", pl));
//    }
//    ParameterList plt;
//    plt.push_back(Parameter("v", "vehicle", me));
//    nfl->push_back(NumericalFluent("capacity", plt));
//
//    if (!numericalFluentCallback(nfl))
//    {
//        printf("WAAAAAAAAAHHHHHHHHH2\n");
//        exit(1);
//        return false;
//    }
//
//    if (DEBUG_MOD)
//        cout << *nfl << endl;
//
//    // TODO get sizes 1 - n-2, size n-1 == package, size n == capcity
//    // 1..n-2 + n == full_load
//    deque<double> packs;
//    for (NumericalFluentList::iterator it = nfl->begin(); it != nfl->end(); it++)
//    {
//        packs.push_back((*it).value);
//    }
//    double cap = packs.back();
//    packs.erase(packs.end() - 1);
//    double pSize = packs.back();
//    packs.erase(packs.end() - 1);
//
//    double sumPacks = 0;
//    for (deque<double>::iterator it = packs.begin(); it != packs.end(); it++)
//    {
//        sumPacks += *it;
//    }
//
//    double loadCap = sumPacks + cap;
//    packs.push_back(pSize);
//
//    bool loadPoss = checkLoading(packs, loadCap);
//
//    if (!loadPoss)
//        cout << "NOT POSSIBLE!!!" << endl;
//
//    return loadPoss;
//}
bool Vehicle_canLoad(const State* currentState,
        const opl::TransportModules::Vehicle* this_pointer,
        const opl::TransportModules::Package* p,
        int relaxed)
{
//    static int count = 0;
//    count++;
//    cout << "Count: " << count << endl;

    // find all packages that are loaded in this vehicle
    const std::map<std::string, Package*>& packages = currentState->getPackages();
    deque<double> packs;
    double sumPacks = 0;
    for (std::map<std::string, Package*>::const_iterator pIt = packages.begin(); pIt != packages.end(); pIt++)
    {
        const Package* package = pIt->second;
        if (package->in(this_pointer))
        {
            packs.push_back(package->size());
            sumPacks += package->size();
        }
    }
    packs.push_back(p->size());
    double loadCap = sumPacks + this_pointer->capacity();
    bool loadPoss = checkLoading(packs, loadCap);

//    if (!loadPoss)
//        cout << "NOT POSSIBLE!!!" << endl;

    return loadPoss;
}

}

}

