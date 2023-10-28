#include "file_loader_backend.hpp"

file_loader_backend::file_loader_backend()
{

}

file_loader_backend::~file_loader_backend()
{
}


int file_loader_backend::set_file_path(const char* _file_str)
{
	close_file();
	if (_file_str == nullptr)
	{
		PX4_INFO("File source is a null pointer!");
		return -1;
	}
	/* store file name */
	strncpy(file_name, _file_str, sizeof(_file_str) - 1);

	/* enforce null termination */
	file_name[sizeof(file_name) - 1] = '\0';
	return 0;
}

int file_loader_backend::read_header(traj_file_header_t& header)
{
	if (open_file() < 0)
	{
		PX4_ERR("Failed to open file");
		return -1;
	}

        if (px4_read(_fd, &header, sizeof(traj_file_header_t)) < 0)
	{
            PX4_ERR("Failed to read header from %d, because %d", _fd, errno);
            return -1;
        }
	return 0;
}

int file_loader_backend::read_data(traj_file_data_t& data)
{
	if (open_file() < 0)
	{
		PX4_ERR("Failed to open file");
		return -1;
	}

        if (px4_read(_fd, &data, sizeof(traj_file_data_t)) < 0)
	{
            PX4_ERR("Failed to read data from %d, because %d", _fd, errno);
            return -1;
        }
	return 0;
}

int file_loader_backend::open_file(void)
{
	if (_fd < 0)
	{
		_fd = px4_open(file_name, O_RDONLY);
		if (_fd < 0)
		{
			PX4_ERR("Can't open file, %d", errno);
			return -1;
		}
		PX4_INFO("Opened file!");
	}

	return 0;
}

int file_loader_backend::close_file(void)
{
	if (_fd > -1)
	{
		px4_close(_fd);
		_fd = -1;
	}
	return 0;
}


