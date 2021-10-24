#include "gtest/gtest.h"
#include "ReadASCII.h"

class ReadASCIITest : public ::testing::Test
{
    private:
        ReadASCII ra;
    protected:
        ReadASCIITest() : lookupTable(ra.lookupTable) { }
        std::vector<ReadASCII::LookupTableColumn>& lookupTable;
        void SetUp() override
        {
        }
        std::vector<std::string> splitLine(const std::string& line, char delim)
        {
            return ra.splitLine(line, delim);
        }
        std::vector<std::vector<std::string>> splitStreamToColumns(std::istream& stream)
        {
            return ra.splitStreamToColumns(stream);
        }
        asynStatus populateLookupTable(const std::vector<std::vector<std::string>>& values)
        {
            return ra.populateLookupTable(values);
        }
};

namespace {

    TEST_F(ReadASCIITest, test_GIVEN_line_WHEN_split_on_space_THEN_line_split) {
        std::vector<std::string> values = splitLine("a b c", ' ');
        EXPECT_EQ(values.size(), 3);
        EXPECT_EQ(values[0], "a");
        EXPECT_EQ(values[1], "b");
        EXPECT_EQ(values[2], "c");
      }

    TEST_F(ReadASCIITest, test_GIVEN_stream_WHEN_split_to_columns_THEN_split) {
        std::stringstream strstm;
        strstm << "SP A B C\n" << "1 2 3 4\n" << "5 6 7 8\n";
        std::vector<std::vector<std::string>> values = splitStreamToColumns(strstm);
        EXPECT_EQ(values.size(), 4);
        EXPECT_EQ(values[0][0], "SP");
        EXPECT_EQ(values[1][1], "2");
      }

    TEST_F(ReadASCIITest, test_GIVEN_split_stream_WHEN_populate_table_THEN_populated) {
        std::stringstream strstm;
        strstm << "SP A B C\n" << "1 2 3 4\n" << "5 6 7 8\n";
        std::vector<std::vector<std::string>> values = splitStreamToColumns(strstm);
        populateLookupTable(values);
        EXPECT_EQ(lookupTable[0].header, "SP");
        EXPECT_EQ(lookupTable[0].values[0], 1);
      }

} // namespace
