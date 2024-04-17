#include <iostream>
#include <string>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

void segmentPlanes(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::string& input_file,
    const std::string& output_dir_planes, const std::string& output_dir_non_planes, const std::string& output_dir_planes_trop_petit)
{
    // Créer l'objet pour l'algorithme de segmentation des plans
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    // Paramètres de l'algorithme de segmentation des plans
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.0005);

    // Segmenter le plan dans le nuage de points
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() < 10)
    {
        std::cerr << "Aucun modèle de plan trouvé dans le nuage de points." << std::endl;
        std::string output_file_non_plan = output_dir_non_planes + "/" + input_file;
        pcl::io::savePCDFileASCII(output_file_non_plan, *cloud);
        std::cerr << "Le nuage de points a été enregistré dans : " << output_file_non_plan << std::endl;
    }
    else
    {
        std::cerr << "Un modèle de plan a été trouvé dans le nuage de points." << std::endl;
        std::cerr << "Coefficients du plan : " << std::endl;
        std::cerr << "    Model coefficients: " << coefficients->values[0] << " "
            << coefficients->values[1] << " "
            << coefficients->values[2] << " "
            << coefficients->values[3] << std::endl;

        if (cloud->size() < 1500)
        {
            std::cerr << "Le nombre de points dans le nuage de points est inférieur au seuil minimum." << std::endl;
            // Enregistrer le nuage de points dans le répertoire des non-plans
            std::string output_file_plan_trop_petit = output_dir_planes_trop_petit + "/" + input_file;
            pcl::io::savePCDFileASCII(output_file_plan_trop_petit, *cloud);
            std::cerr << "Le nuage de points a été enregistré dans : " << output_file_plan_trop_petit << std::endl;
            return;
        }
        else
        {
            // Enregistrer le nuage de points du plan dans le répertoire des plans
            std::string output_file_plan = output_dir_planes + "/" + input_file;
            pcl::io::savePCDFileASCII(output_file_plan, *cloud);
            std::cerr << "Le nuage de points du plan a été enregistré dans : " << output_file_plan << std::endl;
        } 
    }
}

int main(int argc, char** argv)
{
    // Chemin vers le répertoire contenant les fichiers PCD
    std::string directory_path = "C:/Users/etien/source/repos/segmentation/segmentation/marseille_centre_ville_quart_cluster";

    // Répertoire de sortie pour les fichiers contenant les plans détectés
    std::string output_dir_planes = "C:/Users/etien/source/repos/segmentation/segmentation/planes/marseille_centre_ville_quart_cluster";

    // Répertoire de sortie pour les fichiers contenant les plans détectés
    std::string output_dir_planes_trop_petit = "C:/Users/etien/source/repos/segmentation/segmentation/planes_trop_petit/marseille_centre_ville_quart_cluster";

    // Répertoire de sortie pour les fichiers ne contenant pas de plans
    std::string output_dir_non_planes = "C:/Users/etien/source/repos/segmentation/segmentation/non_planes/marseille_centre_ville_quart_cluster";

    // Vérifier si le chemin est valide
    if (!fs::is_directory(directory_path))
    {
        std::cerr << "Le chemin spécifié n'est pas un répertoire valide." << std::endl;
        return 1;
    }

    // Créer les répertoires de sortie s'ils n'existent pas
    fs::create_directories(output_dir_planes);
    fs::create_directories(output_dir_non_planes);
    fs::create_directories(output_dir_planes_trop_petit);

    // Parcourir tous les fichiers dans le répertoire
    for (const auto& entry : fs::directory_iterator(directory_path))
    {
        // Charger le nuage de points à partir du fichier PCD
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(entry.path().string(), *cloud) == -1)
        {
            std::cerr << "Impossible de charger le fichier PCD : " << entry.path().filename() << std::endl;
            continue;
        }

        std::cerr << "Traitement du fichier PCD : " << entry.path().filename() << std::endl;
        segmentPlanes(cloud, entry.path().filename().string(), output_dir_planes, output_dir_non_planes, output_dir_planes_trop_petit);
    }

    return 0;
}
