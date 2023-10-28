#pragma once

#include <matrix/math.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <matrix/math.hpp>

struct traj_file_header_t
{
	uint8_t n_coeffs;
	uint8_t n_int;
	uint8_t n_dofs;
};

struct traj_file_data_t
{
	uint8_t i_int;
	uint8_t i_dof;
	float t_int;
	float coefs[10];
};

class file_loader_backend
{
private:
	char file_name[30] {};
	int _fd = -1;
	int open_file(void);

public:
	int set_file_path(const char* _file_str);
	int read_header(traj_file_header_t& traj_header);

	int read_data(traj_file_data_t& traj_data);
	int close_file(void);

	file_loader_backend();
	~file_loader_backend();
};
