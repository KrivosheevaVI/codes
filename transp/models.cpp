#include <cstdint>
#include <vector>

#include "google/protobuf/duration.pb.h"
#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"

namespace models
{
    struct DataModel
    {
        std::vector<std::vector<double>> distance_matrix;
        std::vector<int64_t> demands;
        std::vector<int64_t> vehicle_capacities;
        int64_t num_vehicles = 0;
        operations_research::RoutingIndexManager::NodeIndex depot{0};
    };
    struct Position
    {
        double x_coord;
        double y_coord;
    };
}
