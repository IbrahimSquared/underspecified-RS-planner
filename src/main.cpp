#include "UnderSpecifiedRSPlanner.hpp"
#include <SFML/Graphics.hpp>
#include <iostream>
#include <random>

using namespace std::numbers;

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
sf::Color getColor(double value) {
  // jet colormap for SFML visualization/plot
  const int color_index = 255 * value;
  double r, g, b;
  if (color_index < 32) {
    r = 0;
    g = 0;
    b = 0.5156 + 0.0156 * color_index;
  } else if (color_index < 96) {
    r = 0;
    g = 0.0156 + 0.9844 * (color_index - 32.0) / 64;
    b = 1;
  } else if (color_index < 158) {
    r = 0.0156 + (color_index - 96.0) / 64;
    g = 1;
    b = 0.9844 - (color_index - 96.0) / 64;
  } else if (color_index < 223) {
    r = 1;
    g = 1 - (color_index - 158.0) / 65;
    b = 0;
  } else {
    r = (2 - (color_index - 223.0) / 32) / 2.0;
    g = 0;
    b = 0;
  }
  return sf::Color(static_cast<sf::Uint8>(r * 255),
                   static_cast<sf::Uint8>(g * 255),
                   static_cast<sf::Uint8>(b * 255));
}

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

  for (int i = 0; i < nx; ++i) {
    for (int j = 0; j < ny; ++j) {
      const double d_normalized = (map[i][j] - min) / (max - min);
      sf::Color color = getColor(d_normalized);
      image.setPixel(i, j, color);
    }
  }

  int number_of_contour_lines = 30;
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
      urs.getOmega(x0, y0, i, j, r, theta0, omega);
      omega_values[i][ny - j - 1] = omega + theta0;
    }
  }
  saveResultImage(omega_values, "omega_values.png");
  return 0;
}
