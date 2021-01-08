#define CATCH_CONFIG_MAIN
#include <nanoplan/nanoplan.h>

#include <catch2/catch.hpp>
#include <string>

TEST_CASE("Priority queue insert works.", "[PriorityQueueWithRemove]") {
  nanoplan::PriorityQueueWithRemove<std::string> pq;
  pq.insert("abc", -1);
  pq.insert("def", 100);
  pq.insert("ghi", -100);
  pq.insert("jkl", 1000);
  REQUIRE(pq.top() == "ghi");
}
