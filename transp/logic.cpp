#include <cstdint>
#include <vector>
#include <fstream>
#include <filesystem>

#include "models.cpp"

#include "google/protobuf/duration.pb.h"
#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"

namespace fs = std::filesystem;

namespace operations_research
{
  void getInput(std::vector<std::vector<double>> &m, int n, std::vector<models::Position> &p)
  {
    for (int l = 0; l < n; l++)
    {
      for (int i = 0; i < n; i++)
      {
        if (l == i)
        {
          m[l][i] = 0;
        }
        else
        {
          m[l][i] = sqrt(pow((p[l].x_coord - p[i].x_coord), 2) + pow((p[l].y_coord - p[i].y_coord), 2));
        }
      }
    }
  };
  void PrintSolution(const models::DataModel &data, const RoutingIndexManager &manager,
                     const RoutingModel &routing, const Assignment &solution)
  {
    int64_t total_distance{0};
    int64_t total_load{0};
    for (int vehicle_id = 0; vehicle_id < data.num_vehicles; ++vehicle_id)
    {
      int64_t index = routing.Start(vehicle_id);
      int64_t route_distance{0};
      int64_t route_load{0};
      std::stringstream route;
      while (routing.IsEnd(index) == false)
      {
        int64_t node_index = manager.IndexToNode(index).value();
        route_load += data.demands[node_index];
        int64_t previous_index = index;
        index = solution.Value(routing.NextVar(index));
        route_distance += routing.GetArcCostForVehicle(previous_index, index,
                                                       int64_t{vehicle_id});
      }
      total_distance += route_distance;
      total_load += route_load;
    }
    std::cout << total_distance << std::endl;
  };
  void VrpCapacity(models::DataModel data_test)
  {
    models::DataModel data = data_test;
    RoutingIndexManager manager(data.distance_matrix.size(), data.num_vehicles,
                                data.depot);
    RoutingModel routing(manager);
    const int transit_callback_index = routing.RegisterTransitCallback(
        [&data, &manager](int64_t from_index, int64_t to_index) -> int64_t
        {
          int from_node = manager.IndexToNode(from_index).value();
          int to_node = manager.IndexToNode(to_index).value();
          return data.distance_matrix[from_node][to_node];
        });
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index);
    const int demand_callback_index = routing.RegisterUnaryTransitCallback(
        [&data, &manager](int64_t from_index) -> int64_t
        {
          int from_node = manager.IndexToNode(from_index).value();
          return data.demands[from_node];
        });
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        int64_t{0},
        data.vehicle_capacities,
        true,
        "Capacity");
    RoutingSearchParameters search_parameters = DefaultRoutingSearchParameters();
    search_parameters.set_first_solution_strategy(
        FirstSolutionStrategy::PATH_CHEAPEST_ARC);
    search_parameters.set_local_search_metaheuristic(
        LocalSearchMetaheuristic::GUIDED_LOCAL_SEARCH);
    search_parameters.mutable_time_limit()->set_seconds(300);
    const Assignment *solution = routing.SolveWithParameters(search_parameters);
    PrintSolution(data, manager, routing, *solution);
  };
  void runApp()
  {
    std::string path = "/Users/georgekobylyanskiy/data";
    auto it = fs::directory_iterator(path);
    std::vector<fs::path> array_path;
    copy_if(fs::begin(it), fs::end(it), std::back_inserter(array_path),
            [](const auto &entry)
            {
              return fs::is_regular_file(entry);
            });
    for (auto &p : array_path)
    {
      std::ifstream new_file_stream;
      new_file_stream.open(p.string());

      std::cout << p.string() << std::endl;

      int64_t N, num_vehicles_one, vehicle_capacities_one;
      new_file_stream >> N >> num_vehicles_one >> vehicle_capacities_one;
      std::vector<int64_t> buffer(num_vehicles_one, vehicle_capacities_one);
      models::DataModel data;
      data.vehicle_capacities = buffer;
      data.num_vehicles = num_vehicles_one;
      std::vector<models::Position> points(N);
      std::vector<int64_t> buf(N);

      for (int i = 0; i < N; i++)
      {
        int64_t demand;
        models::Position p;
        new_file_stream >> demand >> p.x_coord >> p.y_coord;
        points[i] = p;
        buf[i] = demand;
      }

      std::vector<std::vector<double>> matrix(N, std::vector<double>(N));
      getInput(matrix, N, points);
      data.demands = buf;
      data.distance_matrix = matrix;
      VrpCapacity(data);
    }
  };
}
