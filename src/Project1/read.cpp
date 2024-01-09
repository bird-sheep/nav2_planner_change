#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

std::vector<std::vector<int>> readMatrixFromFile(const std::string& filename) {
  std::ifstream file(filename);
  std::vector<std::vector<int>> map;
  std::string line;

  if (file.is_open()) {

        int rows = 3;
        int cols = 2;
        map.resize(3, std::vector<int>(2));

        for (int i = 0; i < rows; ++i) {
          if (std::getline(file, line)) {
            std::istringstream iss(line);
            for (int j = 0; j < cols; ++j) {
              if (!(iss >> map[i][j])) {
                // 处理读取错误或者文件格式不匹配的情况
              }
            }
          }
        }
      }
  return map;
}

int main() {
  std::vector<std::vector<int>> map = readMatrixFromFile("tmp3.txt");

  // 打印读取的矩阵
  for (const auto& row : map) {
    for (int cell : row) {
      std::cout << cell << " ";
    }
    std::cout << std::endl;
  }

  return 0;
}
