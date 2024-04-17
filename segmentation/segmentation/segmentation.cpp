#include <iostream>
#include <limits> // Pour std::numeric_limits
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

int main(int argc, char** argv)
{
    // Charger le nuage de points de la ville de Lyon
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>("C:/Users/etien/Documents/marseille_centre_ville_quart.pcd", *cloud);

    // Créer l'objet KdTree pour la recherche des voisins
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    // Définir les paramètres de l'algorithme DBSCAN
    float tolerance = 0.35; // Tolérance de distance pour la recherche des voisins
    int min_cluster_size = 200; // Taille minimale d'un cluster
    int max_cluster_size = std::numeric_limits<int>::max(); // Taille maximale d'un cluster

    // Appliquer l'algorithme DBSCAN pour extraire les clusters
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(tolerance);
    ec.setMinClusterSize(min_cluster_size);
    ec.setMaxClusterSize(max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    // Dossier de destination
    std::string destination_folder = "C:/Users/etien/source/repos/segmentation/segmentation/marseille_centre_ville_quart_cluster/";

    fs::create_directories(destination_folder);

    // Enregistrer chaque cluster dans un fichier PCD distinct dans le dossier spécifié
    for (size_t i = 0; i < cluster_indices.size(); ++i)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& index : cluster_indices[i].indices)
        {
            cluster_cloud->points.push_back(cloud->points[index]);
        }
        cluster_cloud->width = cluster_cloud->points.size();
        cluster_cloud->height = 1;
        cluster_cloud->is_dense = true;

        std::stringstream ss;
        ss << destination_folder << "cluster_" << i << ".pcd";
        pcl::io::savePCDFile(ss.str(), *cluster_cloud);

        std::cout << "Cluster " << i << " enregistré dans " << ss.str() << std::endl;
    }

    return 0;
}
