#pragma once
#include <cctype>
#include <fstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace utils
{

    // Minimal CSV reader with:
    // - header → column index
    // - quoted fields
    // - comment / blank skipping
    // - typed helpers (bool / int / double)
    class CsvReader
    {
    public:
        CsvReader() = default;

        bool open(const std::string &path)
        {
            file_.open(path);
            if (!file_.is_open())
                return false;

            header_.clear();
            col_index_.clear();

            std::string line;
            if (!std::getline(file_, line))
                return false;

            header_ = parse_line(line);
            for (size_t i = 0; i < header_.size(); ++i)
            {
                trim_inplace(header_[i]);
                col_index_[header_[i]] = static_cast<int>(i);
            }
            return true;
        }

        bool read_row(std::vector<std::string> &out)
        {
            out.clear();
            if (!file_.is_open())
                return false;

            std::string line;
            while (std::getline(file_, line))
            {
                if (is_blank(line))
                    continue;
                if (!line.empty() && line[0] == '#')
                    continue;

                out = parse_line(line);
                if (out.size() < header_.size())
                    out.resize(header_.size());

                for (auto &cell : out)
                    trim_inplace(cell);

                return true;
            }
            return false;
        }

        int col(const std::string &name) const
        {
            auto it = col_index_.find(name);
            if (it == col_index_.end())
                return -1;
            return it->second;
        }

        std::string get(const std::vector<std::string> &row,
                        const std::string &col_name) const
        {
            int idx = col(col_name);
            if (idx < 0 || static_cast<size_t>(idx) >= row.size())
                return "";
            return row[static_cast<size_t>(idx)];
        }

        // ---- Typed helpers (IMPORTANT for CAN map) ----

        static bool to_bool(const std::string &s, bool default_val = false)
        {
            if (s.empty())
                return default_val;
            std::string v = to_lower(s);
            return (v == "true" v == "1" v == "yes");
        }

        static int to_int(const std::string &s, int default_val = 0)
        {
            if (s.empty())
                return default_val;
            return std::stoi(s);
        }

        static uint32_t to_uint32(const std::string &s, uint32_t default_val = 0)
        {
            if (s.empty())
                return default_val;
            // supports hex (0x...)
            return static_cast<uint32_t>(std::stoul(s, nullptr, 0));
        }

        static double to_double(const std::string &s, double default_val = 0.0)
        {
            if (s.empty())
                return default_val;
            // supports scientific notation (1.00E-07)
            return std::stod(s);
        }

    private:
        static bool is_blank(const std::string &s)
        {
            for (char c : s)
                if (!std::isspace(static_cast<unsigned char>(c)))
                    return false;
            return true;
        }

        static std::string to_lower(const std::string &s)
        {
            std::string out;
            out.reserve(s.size());
            for (char c : s)
                out.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
            return out;
        }

        static void trim_inplace(std::string &s)
        {
            size_t b = 0;
            while (b < s.size() && std::isspace(static_cast<unsigned char>(s[b])))
                b++;
            size_t e = s.size();
            while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1])))
                e--;
            s = s.substr(b, e - b);
        }

        static std::vector<std::string> parse_line(const std::string &line)
        {
            std::vector<std::string> fields;
            std::string cur;
            cur.reserve(line.size());

            bool in_quotes = false;
            for (size_t i = 0; i < line.size(); ++i)
            {
                char c = line[i];

                if (in_quotes)
                {
                    if (c == '"')
                    {
                        if (i + 1 < line.size() && line[i + 1] == '"')
                        {
                            cur.push_back('"');
                            ++i;
                        }
                        else
                        {
                            in_quotes = false;
                        }
                    }
                    else
                    {
                        cur.push_back(c);
                    }
                }
                else
                {
                    if (c == '"')
                    {
                        in_quotes = true;
                    }
                    else if (c == ',')
                    {
                        fields.push_back(cur);
                        cur.clear();
                    }
                    else
                    {
                        cur.push_back(c);
                    }
                }
            }
            fields.push_back(cur);
            return fields;
        }

    private:
        std::ifstream file_;
        std::vector<std::string> header_;
        std::unordered_map<std::string, int> col_index_;
    };

} // namespace utils