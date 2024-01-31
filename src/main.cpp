#include "UnderSpecifiedRSPlanner.hpp"
#include <SFML/Graphics.hpp>
#include <random>

using namespace std::numbers;

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void saveResultImage(const std::vector<std::vector<double>> &map,
                     const std::string fileName) {
  const int nx = map.size();
  const int ny = map[0].size();
  sf::Image image;
  image.create(nx, ny);

  double max = 0;
  double min = 1e5;
  for (int i = 0; i < nx; ++i) {
    for (int j = 0; j < ny; ++j) {
      if (map[i][j] > max) {
        max = map[i][j];
      }
      if (map[i][j] < min) {
        min = map[i][j];
      }
    }
  }

  // Define color scale mapping function
  auto getColor = [&](double value) {
    double normalizedValue = (value - min) / (max - min);
    int blueComponent = static_cast<int>(255 * (1 - normalizedValue));
    int redComponent = static_cast<int>(255 * normalizedValue);
    int greenComponent = 0;
    return sf::Color(redComponent, greenComponent, blueComponent);
  };

  for (int i = 0; i < nx; ++i) {
    for (int j = 0; j < ny; ++j) {
      const double d = map[i][j];
      sf::Color color = getColor(d);
      image.setPixel(i, j, color);
    }
  }

  int number_of_contour_lines = 40;
  double stepSize = (max - min) / number_of_contour_lines;
  std::vector<double> contourLevels;
  for (double level = min; level <= max; level += stepSize) {
    contourLevels.push_back(level);
  }
  for (double level : contourLevels) {
    for (int i = 0; i < nx; ++i) {
      for (int j = 0; j < ny; ++j) {
        double value = map[i][j];
        if (std::abs(value - level) <= 0.002 * (max - min)) {
          image.setPixel(i, j, sf::Color::Black);
        }
      }
    }
  }

  image.saveToFile(fileName);
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
int main() {
  const int x0 = 500;
  const int y0 = 500;

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(-pi, pi);
  // const double theta0 = 15 * pi / 180;
  const double theta0 = dis(gen);

  const int nx = 1000;
  const int ny = 1000;
  const double r = 200;

  std::vector<std::vector<double>> omega_values(nx, std::vector<double>(ny));

  UnderSpecifiedRSPlanner urs;

  double omega = 0;
  double d = 0;
  int condition = 0;
  for (int i = 0; i < nx; ++i) {
    for (int j = 0; j < ny; ++j) {
      const double xn = (i - x0) * cos(theta0) + (j - y0) * sin(theta0) + x0;
      const double yn = -(i - x0) * sin(theta0) + (j - y0) * cos(theta0) + y0;

      urs.getOmega(x0, y0, xn, yn, r, omega);
      omega_values[i][ny - j - 1] = omega + theta0;
    }
  }
  saveResultImage(omega_values, "omega_values.png");
  return 0;
}
