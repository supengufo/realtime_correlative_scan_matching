#ifndef TEST_DISTRIBUTION_HPP
#define TEST_DISTRIBUTION_HPP

#include <eigen3/Eigen/Core>
#include <fstream>
#include <yaml-cpp/yaml.h>

namespace cslibs_math {
template<std::size_t Dim>
struct EIGEN_ALIGN16 TestDistribution {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void write(const std::string &path)
    {
        std::ofstream out(path);
        YAML::Emitter yaml;
        yaml << YAML::BeginMap;
        write<Dim,Dim>("covariance", covariance, yaml);
        write<Dim,1>("mean", mean, yaml);
        write<Dim,1>("eigen_values", eigen_values, yaml);
        write<Dim,Dim>("eigen_vectors", eigen_vectors, yaml);
        write<Dim,1>("data", data, yaml);
        yaml << YAML::EndMap;

        out << yaml.c_str();
        out.close();
    }

    template<std::size_t rows, std::size_t cols>
    void write(const std::string &name,
               const std::vector<Eigen::Matrix<double, rows, cols>> &mats,
               YAML::Emitter &yaml)
    {
        yaml << YAML::Key << name << YAML::BeginSeq;
        for(auto &mat : mats) {
            yaml << YAML::BeginMap;
            yaml << YAML::Key << "rows" << YAML::Value << rows;
            yaml << YAML::Key << "cols" << YAML::Value << cols;
            yaml << YAML::Key << "data" << YAML::Value << YAML::BeginSeq;
            for(std::size_t i = 0 ; i < rows; ++i) {
                for(std::size_t j = 0 ; j < cols ; ++j) {
                    yaml << mat(i,j);
                }
            }
            yaml << YAML::EndSeq;
            yaml << YAML::EndMap;
        }
        yaml << YAML::EndSeq;
    }

    template<std::size_t rows, std::size_t cols>
    void write(const std::string &name,
               const Eigen::Matrix<double, rows, cols> &mat,
               YAML::Emitter &yaml)
    {
        yaml << YAML::Key << name << YAML::BeginMap;
        yaml << YAML::Key << "rows" << YAML::Value << rows;
        yaml << YAML::Key << "cols" << YAML::Value << cols;
        yaml << YAML::Key << "data" << YAML::Value << YAML::BeginSeq;
        for(std::size_t i = 0 ; i < rows; ++i) {
            for(std::size_t j = 0 ; j < cols ; ++j) {
                yaml << mat(i,j);
            }
        }
        yaml << YAML::EndSeq;
        yaml << YAML::EndMap;
    }

    void read(const std::string &path)
    {
        YAML::Node yaml = YAML::LoadFile(path);
        assert(yaml.Type() == YAML::NodeType::Map);
        read<Dim,Dim>(yaml["covariance"], covariance);
        read<Dim,1>(yaml["mean"], mean);
        read<Dim,1>(yaml["eigen_values"], eigen_values);
        read<Dim,Dim>(yaml["eigen_vectors"], eigen_vectors);
        read<Dim,1>(yaml["data"], data);
    }

    template<std::size_t rows, std::size_t cols>
    void read(const YAML::Node  &yaml,
              Eigen::Matrix<double, rows, cols> &mat)
    {
        YAML::const_iterator it = yaml["data"].begin();
        for(std::size_t i = 0 ; i < rows; ++i) {
            for(std::size_t j = 0 ; j < cols ; ++j) {
                mat(i,j) = it->as<double>();
                ++it;
            }
        }
    }

    template<std::size_t rows, std::size_t cols>
    void read(const YAML::Node &yaml,
              std::vector<Eigen::Matrix<double, rows, cols>, Eigen::aligned_allocator<Eigen::Matrix<double, rows, cols>>> &mats)
    {
        for(YAML::const_iterator it = yaml.begin() ; it != yaml.end() ; ++it) {
            Eigen::Matrix<double, rows, cols> mat;
            read<rows, cols>(*it, mat);
            mats.emplace_back(mat);
        }
    }

    Eigen::Matrix<double, Dim, Dim>                                                                      covariance;
    Eigen::Matrix<double, Dim,   1>                                                                      mean;
    Eigen::Matrix<double, Dim,   1>                                                                      eigen_values;
    Eigen::Matrix<double, Dim, Dim>                                                                      eigen_vectors;
    std::vector<Eigen::Matrix<double, Dim, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, Dim, 1>>>  data;

};
}

#endif // TEST_DISTRIBUTION_HPP
