#include <nanoplan/nanoplan.h>

#include <catch2/catch.hpp>
#include <string>

TEST_CASE("Basic operations on PriorityQueue.", "[data_structures]") {
  nanoplan::PriorityQueue<std::string, double> pq;
  pq.insert("jkl", 200);
  pq.insert("ghi", 100);
  pq.insert("def", 0);
  pq.insert("abc", -100);

  // Test access to the min-priority value.
  REQUIRE(pq.top() == "abc");

  // Test access to the min-priority.
  REQUIRE(pq.top_priority() == -100);

  // Remove the element with least priority.
  pq.pop();
  REQUIRE(pq.top() == "def");
  REQUIRE(pq.top_priority() == 0);

  // Remove the rest of the elements.
  pq.pop();
  pq.pop();
  pq.pop();
  REQUIRE(pq.empty());
}

TEST_CASE("Basic operations on PriorityQueueWithRemove.", "[data_structures]") {
  nanoplan::PriorityQueueWithRemove<std::string, double> pq;
  pq.insert("jkl", 200);
  pq.insert("ghi", 100);
  pq.insert("def", 0);
  pq.insert("abc", -100);

  // Test access to the min-priority value.
  REQUIRE(pq.top() == "abc");

  // Test access to the min-priority.
  REQUIRE(pq.top_priority() == -100);

  // Remove the element with least priority.
  pq.pop();
  REQUIRE(pq.top() == "def");
  REQUIRE(pq.top_priority() == 0);

  // Remove a couple more elements.
  pq.remove("def");
  pq.remove("ghi");
  REQUIRE(pq.size() == 1);
  REQUIRE(pq.contains("jkl"));

  // Remove the last element.
  pq.pop();
  REQUIRE(pq.empty());
}
