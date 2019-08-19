#ifndef MULTI_ROBOT_MOTION_PLANNING_OFFWRITER_H
#define MULTI_ROBOT_MOTION_PLANNING_OFFWRITER_H

#include <fstream>

#include <cgal_types.h>

class OffWriter {
private:
    std::ofstream m_stream;

public:
    explicit OffWriter(const std::string &file) : m_stream(file) {
    }

    void write(const Workspace &W) {
        std::size_t N = W.size();
        m_stream << "OFF" << std::endl << std::endl;
        m_stream << N << " 1 " << N << std::endl;
        for (auto iter = W.vertices_begin(); iter != W.vertices_end(); ++iter) {
            auto w = *iter;
            m_stream << w.x() << " " << w.y() << " 0.0" << std::endl;
        }
        m_stream << N << " ";
        for (std::size_t i = 0; i < N; i++) {
            m_stream << " " << i;
        }
    }

    ~OffWriter() {
        m_stream.close();
    }
};


#endif //MULTI_ROBOT_MOTION_PLANNING_OFFWRITER_H
