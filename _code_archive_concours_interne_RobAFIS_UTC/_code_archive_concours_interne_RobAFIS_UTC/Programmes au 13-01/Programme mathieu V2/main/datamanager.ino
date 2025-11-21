#include "datamanager.h"

void DataManager::processScenario() {
    Serial.print("TODO");
}

std::string DataManager::processMessage(std::string datagram)
{
  std::string scenario = datagram.substr(0,2);
  // Cas du scénario standard, identifié par le S
  if(scenario == "9S")
  {
      initRobot();
      datagram.erase(0,2);
      std::stringstream ss(datagram);
      std::string frame;
      int semi_colon_counter = 0;
      int color_index = 0;

      while(ss >> std::ws && std::getline(ss, frame, ':'))
      {
        bool is_color = false;
        for (auto color_it = colors.begin(); color_it != colors.end(); ++color_it) {
          if (frame == *color_it)
          {
            COLORTYPES frame_color;
            if (frame == "B") {
                frame_color = COLORTYPES::BLUE;
            } else if (frame == "R") {
                frame_color = COLORTYPES::RED;
            } else if (frame == "J") {
                frame_color = COLORTYPES::YELLOW;
            } else if (frame == "N") {
                frame_color = COLORTYPES::BLACK;
            } else if (frame == "V") {
                frame_color = COLORTYPES::GREEN;
            } else {
                std::cout << "Not a possible color" << std::endl;
                continue;
            }
            color_list.push_back(frame_color);
            color_index++;
            is_color = true;
            std::cout << frame << std::endl;
          }
        }

        if(!is_color)
        {
          frame_list.push_back(frame);
          std::cout << frame << std::endl;
        }

        ++semi_colon_counter;
        if (semi_colon_counter >= 8)
        {
          caseValueConversion();
          return scenario;
        }
      }
  }
  else{ // à modifier pour mettre le scenario detection
    return "9A";
  }
}


void DataManager::initRobot()
{
  current_angle = 0;
  current_case = "A1";
  frame_list.clear();
  converted_frame_list.clear();
  color_list.clear();
}

void DataManager::caseValueConversion() 
{ 
  for (auto frame_it = frame_list.begin(); frame_it != frame_list.end(); ++frame_it) {
    const std::string current_frame = *frame_it;
    int col;
    int row;
    for (int i = 0; i < current_frame.length(); i++) {
      std::cout << "Frame analysée " << current_frame[i] << std::endl;
      if (i == 0)
      {
        switch(current_frame[i]) {
          case 'A': 
          {
            col = 0;
            break;
          }
          case 'B': 
          {
            col = 1;
            break;
          }
          case 'C': 
          {
            col = 2;
            break;
          }
          case 'D': 
          {
            col = 3;
            break;
          }
          default: col = -1; 
        }

        if (col == -1)
        {
          std::cout << "Erreur pour la mesure de la colonne \n";
        }
      }
      else if(i == 1)
      {
        row = current_frame[i] - '0';
        if (row < 1 || row > 5)
        {
          std::cout << "Erreur pour la mesure de la ligne \n";
        }
      }
      else
      {
        std::cout << "Erreur à la conversion du string vers int,, index erroné \n";
      }
    }
    const int new_frame = (col*5 + row);
    std::cout << new_frame << " ";
    converted_frame_list.push_back(new_frame);  
  }
  findingShortestPath();
}

void DataManager::findingShortestPath() {
    Serial.print("Taille initiale de converted_frame_list: ");
    Serial.println(converted_frame_list.size());
    
    // Afficher les valeurs initiales
    Serial.println("Valeurs dans converted_frame_list:");
    for(size_t i = 0; i < converted_frame_list.size(); i++) {
        Serial.print(converted_frame_list[i]);
        Serial.print(" ");
    }
    Serial.println();

    std::vector<int> working_list = converted_frame_list;
    std::sort(working_list.begin(), working_list.end());
    
    Serial.println("Valeurs après tri:");
    for(size_t i = 0; i < working_list.size(); i++) {
        Serial.print(working_list[i]);
        Serial.print(" ");
    }
    Serial.println();

    int best_cost = 1000;
    std::vector<int> best_frame_list;
    std::vector<bool> best_directions;
    std::vector<Movement> best_movements;
    int best_final_direction;
    int best_final_x;
    int best_final_y;

    do {
        for(int dir_combination = 0; dir_combination < (1 << working_list.size()); dir_combination++) {
            std::vector<bool> directions;
            std::vector<Movement> current_movements;
            int temp = dir_combination;
            // On commence orienté vers les colonnes (1 = haut, 3 = bas)
            int current_direction = 1;
            
            Serial.print("Testing combination: ");
            Serial.println(dir_combination);
            
            for(size_t i = 0; i < working_list.size(); i++) {
                directions.push_back(temp & 1);
                temp >>= 1;
            }
            
            int current_position_x = 0;
            int current_position_y = 0;
            int total_cost = 0;
            
            // Pour chaque point
            for(size_t i = 0; i < working_list.size(); i++) {
                Movement mov;
                mov.point = working_list[i];
                mov.columns_first = directions[i];
                
                // Calcul des coordonnées cible
                int target_x = (working_list[i] - 1) / 5;
                int target_y = (working_list[i] - 1) % 5;
                
                Serial.print("Point ");
                Serial.print(mov.point);
                Serial.print(" -> (");
                Serial.print(target_x);
                Serial.print(",");
                Serial.print(target_y);
                Serial.println(")");
                
                mov.dx = target_x - current_position_x;
                mov.dy = target_y - current_position_y;
                
                // Vérification de la rotation initiale nécessaire
                if(mov.columns_first) {
                    // Si on va d'abord en colonnes, on vérifie si on est déjà dans la bonne direction
                    mov.needs_first_rotation = (mov.dy > 0 && current_direction != 1) ||
                                             (mov.dy < 0 && current_direction != 3);
                    // On met à jour la direction après le mouvement en colonnes
                    current_direction = mov.dy > 0 ? 1 : 3;
                } else {
                    // Si on va d'abord en lignes, on vérifie si on doit tourner depuis la direction des colonnes
                    mov.needs_first_rotation = (current_direction == 1 || current_direction == 3);
                    // On met à jour la direction après le mouvement en lignes
                    current_direction = mov.dx > 0 ? 2 : 4;
                }
                
                // Ajout du coût
                if(mov.needs_first_rotation) total_cost++;
                if((mov.columns_first && mov.dx != 0) || (!mov.columns_first && mov.dy != 0)) {
                    total_cost++;  // Rotation intermédiaire seulement si nécessaire
                }
                total_cost += abs(mov.dx) + abs(mov.dy);
                
                // Mise à jour de la direction finale
                if(mov.columns_first) {
                    if(mov.dx != 0) {
                        // Si on a bougé en lignes, on finit orienté dans cette direction
                        current_direction = mov.dx > 0 ? 2 : 4;
                    } else {
                        // Sinon on garde la direction des colonnes
                        current_direction = mov.dy > 0 ? 1 : 3;
                    }
                } else {
                    if(mov.dy != 0) {
                        // Si on a bougé en colonnes, on finit orienté dans cette direction
                        current_direction = mov.dy > 0 ? 1 : 3;
                    } else {
                        // Sinon on garde la direction des lignes
                        current_direction = mov.dx > 0 ? 2 : 4;
                    }
                }
                
                current_position_x = target_x;
                current_position_y = target_y;
                current_movements.push_back(mov);
            }
            
            // Mise à jour si meilleur chemin
            if(total_cost < best_cost) {
                Serial.print("Nouveau meilleur coût trouvé: ");
                Serial.println(total_cost);
                
                best_cost = total_cost;
                best_frame_list = working_list;
                best_directions = directions;
                best_movements = current_movements;
                best_final_direction = current_direction;
                best_final_x = current_position_x;
                best_final_y = current_position_y;
                
                Serial.print("Taille de best_movements: ");
                Serial.println(best_movements.size());
            }
        }
    } while(std::next_permutation(working_list.begin(), working_list.end()));

    // Vérifications finales
    Serial.print("Taille finale de best_movements: ");
    Serial.println(best_movements.size());
    
    if(best_movements.empty()) {
        Serial.println("ERREUR: Aucun mouvement trouvé!");
        return;
    }

    // Réserver la mémoire nécessaire
    converted_frame_list.clear();
    converted_frame_list.reserve(best_frame_list.size());
    directions_list.clear();
    directions_list.reserve(best_directions.size());
    movements.clear();
    movements.reserve(best_movements.size());

    // Sauvegarde des résultats
    converted_frame_list = best_frame_list;
    directions_list = best_directions;
    movements = best_movements;
    final_direction = best_final_direction;
    final_position_number = best_final_x * 5 + best_final_y + 1;

    // Affichage du chemin complet
    Serial.println("\n=== CHEMIN FINAL SAUVEGARDÉ ===");
    Serial.println("Séquence des points à visiter:");
    for(size_t i = 0; i < movements.size(); i++) {
        Serial.print("\nPoint ");
        Serial.print(movements[i].point);
        Serial.println(":");
        
        if(movements[i].needs_first_rotation) {
            Serial.println("  + Rotation initiale");
        }
        
        if(movements[i].columns_first) {
            Serial.print("  + ");
            Serial.print(abs(movements[i].dy));
            Serial.print(" cases en colonnes (");
            Serial.print(movements[i].dy > 0 ? "haut" : "bas");
            Serial.println(")");
            
            if(movements[i].dx != 0) {
                Serial.println("  + Rotation intermédiaire");
            }
            Serial.print("  + ");
            Serial.print(abs(movements[i].dx));
            Serial.print(" cases en lignes (");
            Serial.print(movements[i].dx > 0 ? "droite" : "gauche");
            Serial.println(")");
        } else {
            Serial.print("  + ");
            Serial.print(abs(movements[i].dx));
            Serial.print(" cases en lignes (");
            Serial.print(movements[i].dx > 0 ? "droite" : "gauche");
            Serial.println(")");
            
            if(movements[i].dy != 0) {
                Serial.println("  + Rotation intermédiaire");
            }
            Serial.print("  + ");
            Serial.print(abs(movements[i].dy));
            Serial.print(" cases en colonnes (");
            Serial.print(movements[i].dy > 0 ? "haut" : "bas");
            Serial.println(")");
        }
    }

    // Affichage de la position et direction finales
    Serial.println("\n=== POSITION FINALE ===");
    Serial.print("Position: ");
    Serial.println(final_position_number);  // Position de 1 à 25
    Serial.print("Direction: ");
    switch(final_direction) {
        case 1:
            Serial.println("Nord (haut)");
            break;
        case 2:
            Serial.println("Est (droite)");
            break;
        case 3:
            Serial.println("Sud (bas)");
            break;
        case 4:
            Serial.println("Ouest (gauche)");
            break;
    }
    
    Serial.println("\n=== FIN DU CHEMIN ===");
}

Movement DataManager::moveToNewColor(COLORTYPES target_color, const std::vector<COLORTYPES>& color_positions, int current_position, int current_direction) {
    Movement mov;
    
    // Trouver la position E correspondant à la couleur
    int color_index = -1;
    for(size_t i = 0; i < color_positions.size(); i++) {
        if(color_positions[i] == target_color) {
            color_index = i;
            break;
        }
    }
    
    if(color_index == -1) {
        Serial.println("ERREUR: Couleur non trouvée dans le vecteur!");
        mov.point = 0;  // Indique une erreur
        return mov;
    }
    
    // Calculer la case E correspondante (E1 à E5)
    mov.point = color_index + 21;  // 21 = E1, 22 = E2, etc.
    
    // Position actuelle (passée en paramètre)
    int current_x = (current_position - 1) / 5;
    int current_y = (current_position - 1) % 5;
    
    // Position cible (toujours sur la ligne E = 4)
    int target_x = 4;  // Ligne E
    int target_y = color_index;  // E1 à E5
    
    // Calcul des déplacements nécessaires
    mov.dx = target_x - current_x;
    mov.dy = target_y - current_y;
    
    // On force le mouvement en colonnes d'abord
    mov.columns_first = true;
    
    // Vérification de la rotation initiale nécessaire en fonction de l'orientation passée en paramètre
    if(mov.dy > 0) {
        mov.needs_first_rotation = (current_direction != 1);
    } else if(mov.dy < 0) {
        mov.needs_first_rotation = (current_direction != 3);
    } else {
        mov.needs_first_rotation = false;
    }
    
    return mov;
}

std::vector<Movement> DataManager::getMovements()
{
  return movements;
}

std::vector<bool> DataManager::getDirections()
{
  return directions_list;
}

int DataManager::getFinalPosition()
{
  return final_position_number;
}

int DataManager::getFinalDirection()
{
  return final_direction;
}

std::vector<COLORTYPES> DataManager::getColorList()
{
  return color_list;
}