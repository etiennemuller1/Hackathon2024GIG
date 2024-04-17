#include <iostream>
#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace fs = boost::filesystem;


float getLowestPointHeight(const std::string& input_file) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_file, *cloud) == -1) {
        std::cerr << "Couldn't read file: " << input_file << std::endl;
        return 0.0f;
    }
    if (cloud->points.empty()) {
        std::cerr << "Empty cloud in file: " << input_file << std::endl;
        return 0.0f;
    }

    float lowest_height = std::numeric_limits<float>::max();
    for (const auto& point : cloud->points) {
        if (point.z < lowest_height) {
            lowest_height = point.z;
        }
    }

    return lowest_height;
}

void processPointCloud(const std::string& input_file, const std::string& output_directory, float height) {
    // Charger le nuage de points
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_file, *cloud) == -1) {
        std::cerr << "Couldn't read file: " << input_file << std::endl;
        return;
    }

    // Modifier la coordonnée Z de chaque point à la hauteur prédéfinie
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        cloud->points[i].z = height;
    }

    // Extraire le nom de fichier sans le chemin d'accès
    std::string filename = fs::path(input_file).filename().string();
    std::string output_file = output_directory + "/" + filename;

    // Enregistrer le nuage de points modifié dans un fichier de sortie
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ>(output_file, *cloud, false);

    std::cout << "Modified cloud saved to: " << output_file << std::endl;
}

int main(int argc, char** argv) {

    std::string input_directory = "C:/Users/etien/source/repos/segmentation/segmentation/planes/marseille_centre_ville_quart_cluster";
    std::string output_directory = "C:/Users/etien/source/repos/segmentation/segmentation/footprint/marseille_centre_ville_quart_cluster";


    std::string first_file = input_directory + "/" + fs::directory_iterator(input_directory)->path().filename().string();
    float first_point_height = getLowestPointHeight(first_file);

    // Vérifier si le répertoire de sortie existe, sinon le créer
    if (!fs::exists(output_directory))
        fs::create_directory(output_directory);

    // Parcourir tous les fichiers PCD dans le répertoire d'entrée
    for (const auto& entry : fs::directory_iterator(input_directory)) {
        if (entry.path().extension() == ".pcd") {
            processPointCloud(entry.path().string(), output_directory, first_point_height);
        }
    }

    return 0;
}
