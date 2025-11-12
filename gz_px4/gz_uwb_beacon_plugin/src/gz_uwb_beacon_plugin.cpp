#include "gz_uwb_beacon_plugin/gz_uwb_beacon_plugin.h"

namespace gz
{

	GzUwbBeaconPlugin::GzUwbBeaconPlugin()
	{
		// Inicializar el periodo de actualización
		update_period_ = std::chrono::nanoseconds(0);
		// Initialize ROS2 clock
		ros_clock_ = rclcpp::Clock(RCL_SYSTEM_TIME);
		// Inicializar el generador de números aleatorios
		std::random_device rd;
		random_generator_ = std::default_random_engine(rd());
	}

	void GzUwbBeaconPlugin::Configure(const sim::Entity& _entity, const std::shared_ptr<const sdf::Element>& _sdf,
		sim::EntityComponentManager& _ecm, sim::EventManager& _eventMgr)
	{
		if (!rclcpp::ok())
		{
			rclcpp::init(0, nullptr);
		}

		// Obtener entidad del mundo
		world_entity_ = _ecm.EntityByComponents(sim::components::World());

		// Obtener entidad del tag (se asume el propio modelo como tag)
		tag_entity_ = _entity;

		// Obtener parámetros del plugin
		double update_rate = 25.0;
		std::string beacon_model_name = "";
		int tag_id = 0;
		BeaconMode beacon_mode = DISTANCE;
		double reception_probability = 1.0;
		double beacon_noise_std_eif = 0.0;
		double nlos_soft_wall_width = 0.25;
		double max_db_distance = 30.0;
		double step_db_distance = 0.1;
		if (_sdf->HasElement("update_rate"))
		{
			update_rate = _sdf->Get<double>("update_rate");
		}
		if (_sdf->HasElement("beacon_name"))
		{
			beacon_model_name = _sdf->Get<std::string>("beacon_name");
		}
		if (_sdf->HasElement("tag_id"))
		{
			tag_id = _sdf->Get<int>("tag_id");
		}
		if (_sdf->HasElement("beacon_mode"))
		{
			beacon_mode = static_cast<BeaconMode>(_sdf->Get<int>("beacon_mode"));
		}
		if (_sdf->HasElement("reception_probability"))
		{
			reception_probability = _sdf->Get<double>("reception_probability");
			// Asegurar que esté en el rango válido [0.0, 1.0]
			reception_probability = std::max(0.0, std::min(1.0, reception_probability));
		}
		if (_sdf->HasElement("beacon_noise_std_eif"))
		{
			beacon_noise_std_eif = _sdf->Get<double>("beacon_noise_std_eif");
		}
		if (_sdf->HasElement("nlos_soft_wall_width"))
		{
			nlos_soft_wall_width = _sdf->Get<double>("nlos_soft_wall_width");
		}
		if (_sdf->HasElement("max_db_distance"))
		{
			max_db_distance = _sdf->Get<double>("max_db_distance");
		}

		// Inicializar nodo ROS2
		node_ = std::make_shared<rclcpp::Node>("uwb_beacon_node_" + beacon_model_name);

		// Obtener tasa de actualización
		update_period_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(1.0 / update_rate));
		last_update_time_ = std::chrono::steady_clock::time_point();

		// Inicializar baliza
		auto models = _ecm.EntitiesByComponents(sim::components::Model());
		for (const auto& model_entity : models)
		{
			auto name_comp = _ecm.Component<sim::components::Name>(model_entity);
			if (name_comp && name_comp->Data().find(beacon_model_name) == 0)
			{
				// Baliza encontrada
				// Obtener ID de la baliza (número al final del nombre)
				beacon_params_.id = std::stoi(beacon_model_name.substr(beacon_model_name.length() - 1));

				// Asignar ID del tag
				beacon_params_.tag_id = tag_id;

				// Asignar modo de la baliza
				beacon_params_.mode = beacon_mode;

				// Inicializar parámetros intrínsecos de la baliza
				beacon_params_.noise_std = beacon_noise_std_eif;
				beacon_params_.reception_probability = reception_probability;
				beacon_params_.nlos_soft_wall_width = nlos_soft_wall_width;
				beacon_params_.max_db_distance = max_db_distance;
				beacon_params_.step_db_distance = step_db_distance;

				// Inicializar parámetros de Gazebo
				beacon_params_.model_name = beacon_model_name;
				auto beacon_pose_comp = _ecm.Component<sim::components::Pose>(model_entity);
				beacon_params_.pose = beacon_pose_comp ? beacon_pose_comp->Data() : math::Pose3d();

				// Inicializar medidas de la baliza
				beacon_measurements_.los_type = NLOS;
				beacon_measurements_.distance = 0.0;
				beacon_measurements_.rss = 0.0;
				beacon_measurements_.error_estimation = 0.0;

				// Crear publicador de medidas (solo si el modo es DISTANCE)
				if (beacon_params_.mode == DISTANCE)
				{
					std::string measurement_topic = "/uwb_beacon/" + beacon_params_.model_name + "/measurement";
					beacon_params_.measurement_pub = node_->create_publisher<gz_uwb_beacon_msgs::msg::Measurement>(
						measurement_topic, 10);
					RCLCPP_INFO(node_->get_logger(), "UWB-Beacon-Plugin: Measurement Publishing in %s", measurement_topic.c_str());
				}

				// Crear publicador de EIF Input (solo si el modo es PARALLEL_EIF)
				if (beacon_params_.mode == PARALLEL_EIF)
				{
					std::string eif_input_topic = "/uwb_beacon/eif_input";
					beacon_params_.eif_input_sub = node_->create_subscription<gz_uwb_beacon_msgs::msg::EIFInput>(
						eif_input_topic, 10, std::bind(&GzUwbBeaconPlugin::eifInputCallback, this, std::placeholders::_1));
					RCLCPP_INFO(node_->get_logger(), "UWB-Beacon-Plugin: EIF Input Subscribing in %s", eif_input_topic.c_str());
				}

				// Crear publicador de EIF Output (solo si el modo es PARALLEL_EIF)
				if (beacon_params_.mode == PARALLEL_EIF)
				{
					std::string eif_output_topic = "/uwb_beacon/" + beacon_params_.model_name + "/eif_output";
					beacon_params_.eif_output_pub = node_->create_publisher<gz_uwb_beacon_msgs::msg::EIFOutput>(
						eif_output_topic, 10);
					RCLCPP_INFO(node_->get_logger(), "UWB-Beacon-Plugin: EIF Output Publishing in %s", eif_output_topic.c_str());
				}

				// Crear publicador de datos de marcadores (siempre)
				std::string markers_topic = "/uwb_beacon/" + beacon_params_.model_name + "/markers";
				beacon_params_.markers_pub = node_->create_publisher<visualization_msgs::msg::MarkerArray>(markers_topic, 10);
				RCLCPP_INFO(node_->get_logger(), "UWB-Beacon-Plugin: Anchors Position Publishing in %s", markers_topic.c_str());

				break;
			}
		}

		// Obtener cajas de los modelos a comprobar en el cálculo de intersecciones
		model_boxes_.clear();
		for (auto& model_to_check : models_to_check_)
		{
			// Obtener el modelo
			auto model = _ecm.EntityByComponents(sim::components::Name(model_to_check));
			if (!model)
				continue;

			// Obtener la caja del modelo
			math::AxisAlignedBox model_box = getModelBox(_ecm, model);
			model_boxes_[model_to_check] = model_box;
		}

		// Log
		RCLCPP_INFO(node_->get_logger(),
			"UWB-Beacon-Plugin: Plugin is running. Beacon ID: %d, Tag ID: %d, Beacon Mode: %d",
			beacon_params_.id, tag_id, static_cast<int>(beacon_params_.mode));
	}

	void GzUwbBeaconPlugin::Reset(const sim::UpdateInfo& _info, sim::EntityComponentManager& _ecm)
	{
		RCLCPP_INFO(node_->get_logger(), "UWB-Beacon-Plugin: Resetting Plugin");
		last_update_time_ = std::chrono::steady_clock::now();
	}

	void GzUwbBeaconPlugin::PreUpdate(const sim::UpdateInfo& _info, sim::EntityComponentManager& _ecm)
	{
		// Procesar eventos ROS2
		if (rclcpp::ok())
		{
			rclcpp::spin_some(node_);
		}

		// Verificar si es tiempo de actualizar
		auto current_time = std::chrono::steady_clock::now();
		auto elapsed = current_time - last_update_time_;
		if (elapsed < update_period_)
		{
			return;
		}
		last_update_time_ = current_time;

		// Obtener nueva pose del tag
		auto tag_pose_comp = _ecm.Component<sim::components::Pose>(tag_entity_);
		math::Pose3d tag_pose = tag_pose_comp ? tag_pose_comp->Data() : math::Pose3d();

		// Calcular la distancia real entre la baliza y el tag
		const double initial_ranging = computeDistanceToTag(tag_pose);

		// Comprobar si la distancia supera la distancia máxima de la baliza
		LineOfSight los_type;
		double ranging = initial_ranging;
		if (initial_ranging > beacon_params_.max_db_distance)
		{
			los_type = NLOS;
		}
		// Si la distancia es menor que la distancia máxima de la baliza, calcular el Line-Of-Sight
		else
		{
			los_type = computeLineOfSight(initial_ranging, tag_pose, ranging);
		}

		// Si hay algún Line-Of-Sight
		double distance = HUGE_VAL;
		double rss = 0.0;
		if (los_type != NLOS)
		{
			// Ajustar índice de escenario según el LOS (0 -> LOS, 1 -> NLOS_H, 2 -> NLOS_S)
			int index_scenario = 0;

			if (los_type == NLOS_S)
			{
				index_scenario = 2;
			}
			else if (los_type == NLOS_H)
			{
				index_scenario = 1;
			}

			// Obtener índice de offset de distancia
			int index_ranging_offset = (int)round(ranging / beacon_params_.step_db_distance);

			// Añadir offset a la distancia
			double ranging_with_offset = ranging;
			if (los_type == LOS)
			{
				ranging_with_offset =
					ranging + ranging_offset_[index_ranging_offset][0] / 1000.0;
			}
			else if (los_type == NLOS_S)
			{
				ranging_with_offset =
					ranging + ranging_offset_[index_ranging_offset][1] / 1000.0;
			}

			// Obtener índice de distancia
			int index_ranging = (int)round(ranging_with_offset / beacon_params_.step_db_distance);

			// Computar la distancia según el índice de distancia y de escenario
			distance = computeRandomDistance(ranging_with_offset, ranging_std_[index_ranging][index_scenario]);

			// Computar la potencia de la señal según el índice de distancia y de escenario
			rss = computeRandomPower(rss_mean_[index_ranging][index_scenario], rss_std_[index_ranging][index_scenario]);

			// Si la potencia es menor que el umbral, se considera NLOS
			if (rss < min_power_[index_scenario])
			{
				los_type = NLOS;
				distance = HUGE_VAL;
			}
		}

		// Crear y completar BeaconMeasurements
		BeaconMeasurements beacon_measurements;
		beacon_measurements.distance = distance;
		beacon_measurements.rss = rss;
		beacon_measurements.error_estimation = 0.00393973;
		beacon_measurements.los_type = los_type;

		// Si la baliza es visible y estamos en el modo DISTANCE, publicar medida
		if (los_type != NLOS && beacon_params_.mode == DISTANCE)
		{
			publishMeasurement(beacon_measurements);
		}

		// Publicar marcadores de la baliza
		publishMarkers(tag_pose, beacon_measurements);

		// Actualizar medidas de la baliza bajo lock
		{
			std::lock_guard<std::mutex> lock(beacon_mutex_);

			// Actualizar LOS de la baliza
			beacon_measurements_.los_type = beacon_measurements.los_type;

			// Actualizar medidas
			beacon_measurements_.distance = beacon_measurements.distance;
			beacon_measurements_.rss = beacon_measurements.rss;
			beacon_measurements_.error_estimation = beacon_measurements.error_estimation;
		}
	}

	void GzUwbBeaconPlugin::publishMeasurement(const BeaconMeasurements& beacon_measurements)
	{
		// Generar número aleatorio entre 0 y 1 para simular interferencias
		std::uniform_real_distribution<double> uniform_dist(0.0, 1.0);
		double random_value = uniform_dist(random_generator_);

		// Solo publicar si el valor aleatorio es menor que la probabilidad de recepción
		if (random_value <= beacon_params_.reception_probability)
		{
			gz_uwb_beacon_msgs::msg::Measurement msg;
			msg.header.stamp = ros_clock_.now();
			msg.beacon_id = beacon_params_.id;
			msg.tag_id = beacon_params_.tag_id;
			msg.distance = beacon_measurements.distance;
			msg.rss = beacon_measurements.rss;
			msg.error_estimation = beacon_measurements.error_estimation;

			// Publicar el mensaje
			beacon_params_.measurement_pub->publish(msg);

			RCLCPP_DEBUG(node_->get_logger(), "UWB-Beacon-Plugin: Measurement published for beacon %d (probability: %.3f)",
				beacon_params_.id, random_value);
		}
		else
		{
			RCLCPP_DEBUG(node_->get_logger(), "UWB-Beacon-Plugin: Measurement dropped for beacon %d due to interference (probability: %.3f)",
				beacon_params_.id, random_value);
		}
	}

	void GzUwbBeaconPlugin::publishMarkers(const gz::math::Pose3d& tag_pose, const BeaconMeasurements& beacon_measurements)
	{
		// Crear array de marcadores
		visualization_msgs::msg::MarkerArray markers;

		// Marcador para el cuerpo de la baliza (cilindro central)
		// visualization_msgs::msg::Marker body;
		// body.header.frame_id = "map";
		// body.header.stamp = ros_clock_.now();
		// body.id = 0;
		// body.type = visualization_msgs::msg::Marker::CYLINDER;
		// body.action = visualization_msgs::msg::Marker::ADD;
		// body.pose.position.x = beacon_params_.pose.Pos().X();
		// body.pose.position.y = beacon_params_.pose.Pos().Y();
		// body.pose.position.z = beacon_params_.pose.Pos().Z() - 0.25;
		// body.pose.orientation.x = beacon_params_.pose.Rot().X();
		// body.pose.orientation.y = beacon_params_.pose.Rot().Y();
		// body.pose.orientation.z = beacon_params_.pose.Rot().Z();
		// body.pose.orientation.w = beacon_params_.pose.Rot().W();
		// body.scale.x = 0.25;
		// body.scale.y = 0.25;
		// body.scale.z = 0.5;
		// body.color.a = 1.0;
		// body.color.r = 0.0;
		// body.color.g = 0.0;
		// body.color.b = 1.0;
		// markers.markers.push_back(body);

		// Marcador para la base de la baliza (cilindro más grueso)
		// visualization_msgs::msg::Marker base;
		// base.header = body.header;
		// base.id = 1;
		// base.type = visualization_msgs::msg::Marker::CYLINDER;
		// base.action = visualization_msgs::msg::Marker::ADD;
		// base.pose = body.pose;
		// base.pose.position.z = beacon_params_.pose.Pos().Z() - 0.5;
		// base.scale.x = 0.4;
		// base.scale.y = 0.4;
		// base.scale.z = 0.2;
		// base.color = body.color;
		// markers.markers.push_back(base);

		// Marcador para la esfera superior (origen de la baliza)
		// visualization_msgs::msg::Marker sphere;
		// sphere.header = body.header;
		// sphere.id = 2;
		// sphere.type = visualization_msgs::msg::Marker::SPHERE;
		// sphere.action = visualization_msgs::msg::Marker::ADD;
		// sphere.pose = body.pose;
		// sphere.pose.position.z = beacon_params_.pose.Pos().Z();
		// sphere.scale.x = 0.4;
		// sphere.scale.y = 0.4;
		// sphere.scale.z = 0.4;
		// sphere.color = body.color;
		// markers.markers.push_back(sphere);

		// Marcador para la línea entre la baliza y el vehículo (LOS)
		visualization_msgs::msg::Marker los_line;
		los_line.header.frame_id = "map";
		los_line.header.stamp = ros_clock_.now();
		los_line.id = 3;
		los_line.type = visualization_msgs::msg::Marker::LINE_STRIP;
		los_line.action = visualization_msgs::msg::Marker::ADD;
		los_line.pose.orientation.w = 1.0;
		los_line.scale.x = 0.1;  // Grosor de la línea
		los_line.color.a = 1.0;

		// Añadir los puntos de la línea
		geometry_msgs::msg::Point p1, p2;
		p1.x = beacon_params_.pose.Pos().X();
		p1.y = beacon_params_.pose.Pos().Y();
		p1.z = beacon_params_.pose.Pos().Z();
		p2.x = tag_pose.Pos().X();
		p2.y = tag_pose.Pos().Y();
		p2.z = tag_pose.Pos().Z();
		los_line.points.push_back(p1);
		los_line.points.push_back(p2);

		// Colores de las líneas de visión según el tipo de Line-Of-Sight
		if (beacon_measurements.los_type == LOS)
		{
			// Verde
			los_line.color.r = 0.0;
			los_line.color.g = 1.0;
			los_line.color.b = 0.0;
		}
		else if (beacon_measurements.los_type == NLOS_S)
		{
			// Amarillo
			los_line.color.r = 1.0;
			los_line.color.g = 1.0;
			los_line.color.b = 0.0;
		}
		else if (beacon_measurements.los_type == NLOS_H)
		{
			// Naranja
			los_line.color.r = 1.0;
			los_line.color.g = 0.5;
			los_line.color.b = 0.0;
		}
		else if (beacon_measurements.los_type == NLOS)
		{
			// Rojo
			los_line.color.r = 1.0;
			los_line.color.g = 0.0;
			los_line.color.b = 0.0;
		}

		markers.markers.push_back(los_line);

		// Publicar marcadores
		beacon_params_.markers_pub->publish(markers);
	}

	void GzUwbBeaconPlugin::eifInputCallback(const gz_uwb_beacon_msgs::msg::EIFInput::SharedPtr msg)
	{
		// Generar número aleatorio entre 0 y 1 para simular interferencias
		std::uniform_real_distribution<double> uniform_dist(0.0, 1.0);
		double random_value = uniform_dist(random_generator_);

		// Solo publicar si el valor aleatorio es menor que la probabilidad de recepción
		if (random_value > beacon_params_.reception_probability)
		{
			RCLCPP_DEBUG(node_->get_logger(), "UWB-Beacon-Plugin: EIF Input dropped for beacon %d due to interference (probability: %.3f)",
				beacon_params_.id, random_value);
			return;
		}

		// Copiar medidas de la baliza bajo lock
		BeaconMeasurements beacon_measurements;
		{
			std::lock_guard<std::mutex> lock(beacon_mutex_);
			beacon_measurements = beacon_measurements_;
		}

		// Si no hay LOS
		if (beacon_measurements.los_type == NLOS)
		{
			RCLCPP_DEBUG(node_->get_logger(), "UWB-Beacon-Plugin: NLOS. No EIF Input");
			return;
		}

		// Obtener datos relevantes
		const auto& mu_msg = msg->mu;
		const auto& mu_predicted_msg = msg->mu_predicted;
		const double& z = beacon_measurements.distance;
		const double& beacon_noise_std = beacon_params_.noise_std;
		const auto& beacon_position_gz = beacon_params_.pose.Pos();

		// Convertir mensajes a Eigen
		const auto mu = Eigen::Vector3d(mu_msg[0], mu_msg[1], mu_msg[2]);
		const auto mu_predicted = Eigen::Vector3d(mu_predicted_msg[0], mu_predicted_msg[1], mu_predicted_msg[2]);
		const auto beacon_position = Eigen::Vector3d(beacon_position_gz.X(), beacon_position_gz.Y(), beacon_position_gz.Z());

		// Calcular matriz y vector de información parcial (asociado a esta baliza)
		const Eigen::Matrix<double, 1, 3> H = jacobian_H(mu, beacon_position);
		// Calcular ruido de medición estandar de cada baliza
		const double Q = noiseModel_Q(beacon_noise_std);
		// Calcular innovación
		const double y = z - function_h(mu, beacon_position) + H * mu_predicted;
		// Calcular matriz de información parcial
		double Q_inv;
		if (Q > 1e-6)
			Q_inv = 1.0 / Q;
		else
			Q_inv = HUGE_VAL;
		const Eigen::Matrix3d omega = H.transpose() * Q_inv * H;
		// Calcular vector de información parcial (TODO: Revisar tamaño de matrices)
		const Eigen::Vector3d xi = H.transpose() * Q_inv * y;

		// Publicar EIF Output
		gz_uwb_beacon_msgs::msg::EIFOutput output_msg;
		output_msg.header.stamp = ros_clock_.now();
		output_msg.beacon_id = beacon_params_.id;
		output_msg.tag_id = beacon_params_.tag_id;
		output_msg.omega[0] = omega(0, 0);
		output_msg.omega[1] = omega(0, 1);
		output_msg.omega[2] = omega(0, 2);
		output_msg.omega[3] = omega(1, 0);
		output_msg.omega[4] = omega(1, 1);
		output_msg.omega[5] = omega(1, 2);
		output_msg.omega[6] = omega(2, 0);
		output_msg.omega[7] = omega(2, 1);
		output_msg.omega[8] = omega(2, 2);
		output_msg.xi[0] = xi(0);
		output_msg.xi[1] = xi(1);
		output_msg.xi[2] = xi(2);
		beacon_params_.eif_output_pub->publish(output_msg);
	}

	Eigen::Matrix<double, 1, 3> GzUwbBeaconPlugin::jacobian_H(const Eigen::Vector3d& mu, const Eigen::Vector3d& beacon_position)
	{
		// Jacobiano de h de una sola baliza
		// en total dimensiones : 1 x 3
		// Implementación del Jacobiano H para una baliza
		Eigen::Matrix<double, 1, 3> H;
		H.setZero();

		// Calcular la distancia euclidiana entre la posición estimada y la baliza
		double distance = sqrt(
			pow(mu.x() - beacon_position.x(), 2) +
			pow(mu.y() - beacon_position.y(), 2) +
			pow(mu.z() - beacon_position.z(), 2)
		);

		// Evitar división por cero
		if (distance < 1e-6) {
			return H;
		}

		// Calcular cada componente del Jacobiano
		H(0, 0) = (mu.x() - beacon_position.x()) / distance;
		H(0, 1) = (mu.y() - beacon_position.y()) / distance;
		H(0, 2) = (mu.z() - beacon_position.z()) / distance;

		return H;
	}

	double GzUwbBeaconPlugin::noiseModel_Q(const double& noise_std)
	{
		// Modelo de ruido de medición para una baliza :
		// Tamaño: n x n
		// Si se considera que el error de medición es un ruido gaussiano con varianza constante,
		// que todas las mediciones son independientes entre sí y que todas las balizas
		// tienen la misma varianza del ruido de medición
		return noise_std * noise_std;
	}

	double GzUwbBeaconPlugin::function_h(const Eigen::Vector3d& mu, const Eigen::Vector3d& beacon_position)
	{
		// Función h para una sola baliza
		// Calcula la distancia euclidiana entre la posición estimada (mu) y la posición de la baliza
		double h = 0.0;

		// Calcular la distancia euclidiana
		h = sqrt(
			pow(mu.x() - beacon_position.x(), 2) +
			pow(mu.y() - beacon_position.y(), 2) +
			pow(mu.z() - beacon_position.z(), 2)
		);

		return h;
	}

	double GzUwbBeaconPlugin::computeDistanceToTag(const gz::math::Pose3d& tag_pose)
	{
		// Devolver distancia de la baliza y el tag
		return tag_pose.Pos().Distance(beacon_params_.pose.Pos());
	}

	GzUwbBeaconPlugin::LineOfSight GzUwbBeaconPlugin::computeLineOfSight(const double& distance, const gz::math::Pose3d& tag_pose, double& distance_after_rebounds)
	{
		// Inicializar variable como LOS
		LineOfSight los_type = LOS;
		distance_after_rebounds = distance;

		// Obtener posición y orientación del tag y de la baliza
		const math::Vector3d tag_position = tag_pose.Pos();
		const math::Quaternion tag_orientation = tag_pose.Rot();
		const math::Vector3d beacon_position = beacon_params_.pose.Pos();

		// Comprobar si hay un obstáculo entre la baliza y el tag
		double distance_to_obstacle_from_tag = 0.0;
		std::string obstacle_name_1 = "";
		obstacle_name_1 = getIntersection(tag_position, beacon_position, distance_to_obstacle_from_tag);
		if (obstacle_name_1.compare("") == 0)
		{
			// No hay obstáculo entre la baliza y el tag, se usa el modelo Line-Of-Sight
			los_type = LOS;
			distance_after_rebounds = distance;
			RCLCPP_DEBUG(node_->get_logger(), "UWB-Beacon-Plugin: LOS. No obstacles between tag and anchor");
			return los_type;
		}

		// Hay un obstáculo entre la baliza y el tag, se usa el modelo Non-Line-Of-Sight
		// Se usa un rayo adicional para medir la distancia desde la baliza al tag, 
		// para conocer el ancho de las paredes
		double distance_to_obstacle_from_beacon = 0.0;
		std::string obstacle_name_2 = "";
		obstacle_name_2 = getIntersection(beacon_position, tag_position, distance_to_obstacle_from_beacon);

		// Comprobar ancho de la pared
		// Si es menor o igual al parámetro y los dos modelos coinciden, se usa el modelo NLOS_S
		double wall_width = distance - distance_to_obstacle_from_tag - distance_to_obstacle_from_beacon;
		if (wall_width <= beacon_params_.nlos_soft_wall_width && obstacle_name_1.compare(obstacle_name_2) == 0)
		{
			// Se usa el modelo Non-Line-Of-Sight - Soft
			los_type = NLOS_S;
			distance_after_rebounds = distance;
			RCLCPP_DEBUG(node_->get_logger(), "UWB-Beacon-Plugin: NLOS_S. Wall width: %f", wall_width);
			return los_type;
		}

		// La pared es más ancha que el parámetro, se comprueban los rebotes
		// Obtener los ángulos de Euler del tag
		tf2::Quaternion q(tag_orientation.X(), tag_orientation.Y(), tag_orientation.Z(), tag_orientation.W());
		tf2::Matrix3x3 m(q);
		double roll, pitch, current_yaw;
		m.getRPY(roll, pitch, current_yaw);

		// Calcular el barrido de ángulos de prueba
		double start_angle = current_yaw;
		double arc = 3.0 * M_PI / 2.0;
		int num_angles_to_test_by_side = 30;
		double increment_angle = arc / num_angles_to_test_by_side;
		int total_number_angles_to_test = 1 + 2 * num_angles_to_test_by_side;
		double angles_to_test[total_number_angles_to_test];
		angles_to_test[0] = start_angle;
		for (int i = 1; i < total_number_angles_to_test; ++i)
		{
			double angle_to_test;
			if (i % 2 == 0)
			{
				angle_to_test = start_angle - (i / 2) * increment_angle;
			}
			else
			{
				angle_to_test = start_angle + (i - (i - 1) / 2) * increment_angle;
			}
			angles_to_test[i] = angle_to_test;
		}

		// Buscamos un rebote para llegar a la baliza desde el tag
		bool end = false;
		double max_distance = 30.0;
		double step_floor = 1.0;
		double start_floor_distance_check = 2.0;
		int num_steps_floor = 6;
		int index_ray = 0;
		bool found_nlos_h = false;
		double distance_nlos_hard = 0;
		int current_floor_distance = 0;
		while (!end)
		{
			// Calcular un punto perteneciente al rayo a trazar
			double current_angle = angles_to_test[index_ray];

			double x = tag_position.X() + max_distance * cos(current_angle);
			double y = tag_position.Y() + max_distance * sin(current_angle);
			double z = tag_position.Z();

			if (current_floor_distance > 0)
			{
				double tan_angle_floor =
					(start_floor_distance_check + step_floor * (current_floor_distance - 1)) / tag_position.Z();
				double angle_floor = atan(tan_angle_floor);

				double h = sin(angle_floor) * max_distance;

				double horizontal_distance = sqrt(max_distance * max_distance - h * h);

				x = tag_position.X() + horizontal_distance * cos(current_angle);
				y = tag_position.Y() + horizontal_distance * sin(current_angle);
				z = -1.0 * (h - tag_position.Z());

			}

			math::Vector3d ray_point(x, y, z);

			// Obtener intersección entre el tag y el punto del rayo
			std::string obstacle_name_3 = "";
			double distance_to_rebound = 0.0;
			obstacle_name_3 = getIntersection(tag_position, ray_point, distance_to_rebound);

			// Si hay intersección
			if (obstacle_name_3.compare("") != 0)
			{
				// Calcular punto de colisión		
				math::Vector3d collision_point(
					tag_position.X() + distance_to_rebound * cos(current_angle),
					tag_position.Y() + distance_to_rebound * sin(current_angle),
					tag_position.Z());

				if (current_floor_distance > 0.0)
				{
					collision_point.Set(
						tag_position.X() + distance_to_rebound * cos(current_angle),
						tag_position.Y() + distance_to_rebound * sin(current_angle),
						0.0);
				}

				// Obtener intersección entre el punto de colisión y la baliza
				std::string obstacle_name_4 = "";
				double distance_to_final_obstacle = 0.0;
				obstacle_name_4 = getIntersection(collision_point, beacon_position, distance_to_final_obstacle);

				// Si no hay intersección
				if (obstacle_name_4.compare("") == 0)
				{
					// Llegamos a la baliza después de un rebote
					distance_to_final_obstacle = beacon_position.Distance(collision_point);
					if (distance_to_rebound + distance_to_final_obstacle <= beacon_params_.max_db_distance)
					{
						found_nlos_h = true;

						// Buscamos el rebote más corto
						if (distance_nlos_hard < 0.1)
						{
							distance_nlos_hard = distance_to_rebound + distance_to_final_obstacle;
						}
						else if (distance_nlos_hard > distance_to_rebound + distance_to_final_obstacle)
						{
							distance_nlos_hard = distance_to_rebound + distance_to_final_obstacle;
						}
					}
				}
			}

			if (index_ray < total_number_angles_to_test - 1)
			{
				index_ray += 1;
			}
			else
			{
				if (current_floor_distance < num_steps_floor)
				{
					current_floor_distance += 1;
					index_ray = 0;

				}
				else
				{
					end = true;
				}

			}
		}

		// Si se encontró NLOS_H
		if (found_nlos_h)
		{
			// Usamos el modelo Non-Line-Of-Sight - Hard con distancia = distance_nlos_hard
			los_type = NLOS_H;
			distance_after_rebounds = distance_nlos_hard;
			RCLCPP_DEBUG(node_->get_logger(), "UWB-Beacon-Plugin: NLOS_H. Distance after rebounds: %f", distance_after_rebounds);
			return los_type;
		}
		else
		{
			// No podemos llegar a la baliza, no hay ranging
			los_type = NLOS;
			distance_after_rebounds = HUGE_VAL;
			RCLCPP_DEBUG(node_->get_logger(), "UWB-Beacon-Plugin: NLOS. No way to reach the anchor");
			return los_type;
		}

		return los_type;
	}

	double GzUwbBeaconPlugin::computeRandomDistance(const double& ranging_mean, const double& ranging_std)
	{
		// Crear distribución normal para la medida de distancia
		std::normal_distribution<double> distribution_ranging(ranging_mean * 1000.0, ranging_std);
		return distribution_ranging(random_generator_) / 1000.0;
	}

	double GzUwbBeaconPlugin::computeRandomPower(const double& rss_mean, const double& rss_std)
	{
		// Crear distribución normal para la potencia de la señal
		std::normal_distribution<double> distribution_rss(rss_mean, rss_std);
		return distribution_rss(random_generator_);
	}

	std::string GzUwbBeaconPlugin::getIntersection(const math::Vector3d& point1, const math::Vector3d& point2, double& distance)
	{
		std::string obstacle_name = "";

		// Calcula dirección y longitud del rayo
		math::Vector3d direction = point2 - point1;
		double ray_length = direction.Length();
		if (ray_length < 1e-6)
		{
			return obstacle_name;
		}

		// Normalizar dirección
		direction = direction / ray_length;

		// Distancia más corta al rayo
		distance = ray_length;

		// Iterar sobre las cajas de los modelos a comprobar
		for (auto& [model_name, model_box] : model_boxes_)
		{
			// Comprobar si el rayo intersecta con la caja y la distancia es menor que la distancia más corta encontrada
			auto [intersects, hit_distance, hit_point] = model_box.Intersect(point1, direction, 0.0, ray_length);
			if (intersects && hit_distance < distance)
			{
				distance = hit_distance;
				obstacle_name = model_name;
			}
		}

		return obstacle_name;
	}

	math::AxisAlignedBox GzUwbBeaconPlugin::getModelBox(sim::EntityComponentManager& _ecm, sim::Entity& model)
	{
		math::AxisAlignedBox result;
		bool first_box = true;

		// Obtener la pose del modelo
		auto model_pose_comp = _ecm.Component<sim::components::Pose>(model);
		if (!model_pose_comp)
			return result;

		math::Pose3d model_pose = model_pose_comp->Data();

		// Obtener todos los links del modelo
		auto links = _ecm.ChildrenByComponents(model, sim::components::Link());
		if (links.empty())
			return result;

		// Para cada link, obtener todas las colisiones
		for (const auto& link : links)
		{
			// Obtener la pose del link relativa al modelo
			auto link_pose_comp = _ecm.Component<sim::components::Pose>(link);
			if (!link_pose_comp)
				continue;

			math::Pose3d link_pose_rel = link_pose_comp->Data();

			// Calcular la pose absoluta del link en coordenadas del mundo
			math::Pose3d link_pose_abs = model_pose * link_pose_rel;

			// Obtener todas las colisiones para este link
			auto collisions = _ecm.ChildrenByComponents(link, sim::components::Collision());
			if (collisions.empty())
				continue;

			// Procesar cada colisión
			for (const auto& collision : collisions)
			{
				// Obtener la pose de la colisión relativa al link
				auto collision_pose_comp = _ecm.Component<sim::components::Pose>(collision);
				math::Pose3d collision_pose_rel = collision_pose_comp ?
					collision_pose_comp->Data() : math::Pose3d();

				// Calcular la pose absoluta de la colisión en coordenadas del mundo
				math::Pose3d collision_pose_abs = link_pose_abs * collision_pose_rel;

				// Obtener la geometría de la colisión
				auto geom_comp = _ecm.Component<sim::components::Geometry>(collision);
				if (!geom_comp)
					continue;

				auto geom_data = geom_comp->Data();
				if (geom_data.Type() == sdf::GeometryType::BOX)
				{
					// Obtener el tamaño de la caja
					const sdf::Box* box_shape_sdf = geom_data.BoxShape();
					if (!box_shape_sdf)
						continue;

					math::Vector3d box_size = box_shape_sdf->Size();

					// Crear los vértices de la caja en coordenadas locales
					std::vector<math::Vector3d> vertices;
					vertices.reserve(8);
					for (int i = 0; i < 8; ++i)
					{
						double x = ((i & 1) ? 0.5 : -0.5) * box_size.X();
						double y = ((i & 2) ? 0.5 : -0.5) * box_size.Y();
						double z = ((i & 4) ? 0.5 : -0.5) * box_size.Z();
						vertices.push_back(math::Vector3d(x, y, z));
					}

					// Transformar vértices a coordenadas del mundo y expandir la caja
					math::AxisAlignedBox box;
					for (const auto& vertex : vertices)
					{
						// Transformar el vértice a coordenadas del mundo
						math::Vector3d world_vertex = collision_pose_abs.Pos() +
							collision_pose_abs.Rot().RotateVector(vertex);

						// En el primer vértice de la primera caja, inicializar la caja
						if (first_box && &vertex == &vertices.front())
						{
							box.Min() = world_vertex;
							box.Max() = world_vertex;
							first_box = false;
						}
						else
						{
							// Expandir la caja para incluir este vértice
							box.Min().X() = std::min(box.Min().X(), world_vertex.X());
							box.Min().Y() = std::min(box.Min().Y(), world_vertex.Y());
							box.Min().Z() = std::min(box.Min().Z(), world_vertex.Z());

							box.Max().X() = std::max(box.Max().X(), world_vertex.X());
							box.Max().Y() = std::max(box.Max().Y(), world_vertex.Y());
							box.Max().Z() = std::max(box.Max().Z(), world_vertex.Z());
						}
					}

					// Combinar con la caja resultante
					if (first_box)
					{
						result = box;
						first_box = false;
					}
					else
					{
						result.Min().X() = std::min(result.Min().X(), box.Min().X());
						result.Min().Y() = std::min(result.Min().Y(), box.Min().Y());
						result.Min().Z() = std::min(result.Min().Z(), box.Min().Z());

						result.Max().X() = std::max(result.Max().X(), box.Max().X());
						result.Max().Y() = std::max(result.Max().Y(), box.Max().Y());
						result.Max().Z() = std::max(result.Max().Z(), box.Max().Z());
					}
				}
			}
		}

		return result;
	}

	// Registrar el plugin con Gazebo
	GZ_ADD_PLUGIN(
		GzUwbBeaconPlugin,
		sim::System,
		GzUwbBeaconPlugin::ISystemConfigure,
		GzUwbBeaconPlugin::ISystemPreUpdate,
		GzUwbBeaconPlugin::ISystemReset
	)

} // namespace gz

