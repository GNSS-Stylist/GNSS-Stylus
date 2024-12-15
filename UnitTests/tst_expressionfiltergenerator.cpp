/*
    tst_expressionfiltergenerator.cpp (part of GNSS-Stylus)
    Copyright (C) 2024-present Pasi Nuutinmaki (gnssstylist<at>sci<dot>fi)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "tst_expressionfiltergenerator.h"
#include "../PostProcessing/Lidar/PointFilter/expressionfiltergenerator.h"
#include "../PostProcessing/Lidar/PointFilter/expressionfilter_base.h"

TestExpressionFilterGenerator::TestExpressionFilterGenerator()
{
}

TestExpressionFilterGenerator::~TestExpressionFilterGenerator()
{
}

void TestExpressionFilterGenerator::initTestCase()
{
}

void TestExpressionFilterGenerator::cleanupTestCase()
{
}

using EFG = PointFilter::ExpressionFilterGenerator;

void TestExpressionFilterGenerator::validDefinitions_NoConvexHulls()
{
    QString plainText =
        "/*First device: */\n"
        "RPLiDaR\n"
        "{\n"
        "lidar.rplidar.quality\n"
        "}\n"
        "{\n"
        "lidar.rplidar.quality_indexed(5)\n"
        "}\n"
        "mid360 1.2.3.4 // <- Second device\n"
        "{0.5 // BTW: TinyExpr++ seems to need this newline -> \n}{/*Zero point three:*/ 0.3}"
        "/* Third device: */mid360 11.22.33.44\n"
        "{ lidar.coord_indexed.x(0) }\n"
        "{lidar.mid360.properties}\n";

    try
    {
        static QMap<EFG::Device, std::shared_ptr<PointFilter::ExpressionFilter_Base> > filters;

        filters = EFG::generateMap(plainText);

        QCOMPARE(filters.count(), 3);

        auto filter = filters.begin();

        QCOMPARE(filter.key().type, EFG::Device::DT_RPLIDAR);
        QCOMPARE(filter.value()->getExpression_Filter(), "\nlidar.rplidar.quality\n");
        QCOMPARE(filter.value()->getExpression_Quality(), "\nlidar.rplidar.quality_indexed(5)\n");

        filter++;
        QCOMPARE(filter.key().type, EFG::Device::DT_LIVOX_MID360);
        QCOMPARE(filter.value()->getExpression_Filter(), "0.5 // BTW: TinyExpr++ seems to need this newline -> \n");
        QCOMPARE(filter.value()->getExpression_Quality(), "/*Zero point three:*/ 0.3");

        filter++;
        QCOMPARE(filter.key().type, EFG::Device::DT_LIVOX_MID360);
        QCOMPARE(filter.value()->getExpression_Filter(), " lidar.coord_indexed.x(0) ");
        QCOMPARE(filter.value()->getExpression_Quality(), "lidar.mid360.properties");
    }
    catch (EFG::Issue issue)
    {
        QFAIL("Should not throw this exception.");
    }
    catch (...)
    {
        QFAIL("Should not throw any exception.");
    }
}

void TestExpressionFilterGenerator::validDefinitions_ConvexHulls()
{
    QVector<PointFilter::ExpressionFilter_Base::ConvexHullFilter> convexHullFilters;

    PointFilter::ExpressionFilter_Base::ConvexHullFilter firstFilter { .Name = "first", .filter = ConvexHull::Filter() };
    PointFilter::ExpressionFilter_Base::ConvexHullFilter secondFilter { .Name = "second", .filter = ConvexHull::Filter() };
    PointFilter::ExpressionFilter_Base::ConvexHullFilter thirdFilter { .Name = "tHiRd", .filter = ConvexHull::Filter() };

    convexHullFilters.push_back(firstFilter);
    convexHullFilters.push_back(secondFilter);
    convexHullFilters.push_back(thirdFilter);

    QString plainText =
        "/*First device: */\n"
        "RPLiDaR\n"
        "{\n"
        "chull_first\n"
        "}\n"
        "{\n"
        "lidar.in_convex_hull(chull_second, 1) /* foobar */ \n"
        "}\n"
        "mid360 1.2.3.4 // <- Second device\n"
        "{0.5}{0.3}"
        "/* Third device: */mid360 11.22.33.44\n"
        "{ ned.in_convex_hull_indexed(chull_third, 1, -2) }\n"
        "{rig.in_convex_hull(chull_first,0)}\n";

    try
    {
        static QMap<EFG::Device, std::shared_ptr<PointFilter::ExpressionFilter_Base> > filters;

        filters = EFG::generateMap(plainText,convexHullFilters);

        QCOMPARE(filters.count(), 3);

        auto filter = filters.begin();

        QCOMPARE(filter.key().type, EFG::Device::DT_RPLIDAR);
        QCOMPARE(filter.value()->getExpression_Filter(), "\nchull_first\n");
        QCOMPARE(filter.value()->getExpression_Quality(), "\nlidar.in_convex_hull(chull_second, 1) /* foobar */ \n");

        filter++;
        QCOMPARE(filter.key().type, EFG::Device::DT_LIVOX_MID360);
        QCOMPARE(filter.value()->getExpression_Filter(), "0.5");
        QCOMPARE(filter.value()->getExpression_Quality(), "0.3");

        filter++;
        QCOMPARE(filter.key().type, EFG::Device::DT_LIVOX_MID360);
        QCOMPARE(filter.value()->getExpression_Filter(), " ned.in_convex_hull_indexed(chull_third, 1, -2) ");
        QCOMPARE(filter.value()->getExpression_Quality(), "rig.in_convex_hull(chull_first,0)");
    }
    catch (EFG::Issue issue)
    {
        QFAIL("Should not throw this exception.");
    }
    catch (...)
    {
        QFAIL("Should not throw any exception.");
    }
}

void TestExpressionFilterGenerator::error_DuplicateDevices()
{
    QString plainText =
        "/*First device: */\n"
        "RPLiDaR\n"
        "{\n"
        "lidar.rplidar.quality\n"
        "}\n"
        "{\n"
        "lidar.rplidar.quality_indexed(5)\n"
        "}\n"
        "mid360 1.2.3.4 // <- Second device\n"
        "{0.5 // BTW: TinyExpr++ seems to need this newline -> \n}{/*Zero point three:*/ 0.3}"
        "/* Third device: */rplidar\n"
        "{ lidar.coord_indexed.x(0) }\n"
        "{1}\n";

    try
    {
        static QMap<EFG::Device, std::shared_ptr<PointFilter::ExpressionFilter_Base> > filters;

        filters = EFG::generateMap(plainText);

        QFAIL("Should throw an exception");
    }
    catch (EFG::Issue issue)
    {
        QCOMPARE(issue.text, "Duplicate device: \"rplidar\".");
    }
    catch (...)
    {
        QFAIL("Should not throw any other exception.");
    }

    plainText =
        "/*First device: */\n"
        "RPLiDaR\n"
        "{\n"
        "lidar.rplidar.quality\n"
        "}\n"
        "{\n"
        "lidar.rplidar.quality_indexed(5)\n"
        "}\n"
        "mid360 1.2.3.4 // <- Second device\n"
        "{0.5 // BTW: TinyExpr++ seems to need this newline -> \n}{/*Zero point three:*/ 0.3}"
        "/* Third device: */mid360 1.2.3.4 \n"
        "{ lidar.coord_indexed.x(0) }\n"
        "{lidar.mid360.properties}\n";

    try
    {
        static QMap<EFG::Device, std::shared_ptr<PointFilter::ExpressionFilter_Base> > filters;

        filters = EFG::generateMap(plainText);

        QFAIL("Should throw an exception");
    }
    catch (EFG::Issue issue)
    {
        QCOMPARE(issue.text, "Duplicate device: \"mid360 1.2.3.4\".");
    }
    catch (...)
    {
        QFAIL("Should not throw any other exception.");
    }
}

void TestExpressionFilterGenerator::error_UnknownDeviceType()
{
    QString plainText =
        "/*First device: */\n"
        "RPLiDaR\n"
        "{\n"
        "lidar.rplidar.quality\n"
        "}\n"
        "{\n"
        "lidar.rplidar.quality_indexed(5)\n"
        "}\n"
        "superlidar 1.2.3.4 // <- Second device\n"
        "{0.5 // BTW: TinyExpr++ seems to need this newline -> \n}{/*Zero point three:*/ 0.3}"
        "/* Third device: */rplidar\n"
        "{ lidar.coord_indexed.x(0) }\n"
        "{1}\n";

    try
    {
        static QMap<EFG::Device, std::shared_ptr<PointFilter::ExpressionFilter_Base> > filters;

        filters = EFG::generateMap(plainText);

        QFAIL("Should throw an exception");
    }
    catch (EFG::Issue issue)
    {
        QCOMPARE(issue.text, "Unknown device type: \"superlidar\".");
    }
    catch (...)
    {
        QFAIL("Should not throw any other exception.");
    }
}


void TestExpressionFilterGenerator::error_InvalidMid360IP()
{
    QString plainText =
        "/*First device: */\n"
        "mid360 1.2.3.4\n"
        "{0.5 // BTW: TinyExpr++ seems to need this newline -> \n}{/*Zero point three:*/ 0.3}"
        "/* Second device: */mid360 \n"
        "{ lidar.coord_indexed.x(0) }\n"
        "{1}\n";

    try
    {
        static QMap<EFG::Device, std::shared_ptr<PointFilter::ExpressionFilter_Base> > filters;

        filters = EFG::generateMap(plainText);

        QFAIL("Should throw an exception");
    }
    catch (EFG::Issue issue)
    {
        QCOMPARE(issue.text, "IP address needed for device type mid360.");
    }
    catch (...)
    {
        QFAIL("Should not throw any other exception.");
    }

    plainText =
        "/*First device: */\n"
        "mid360 1.2.3.4\n"
        "{0.5 // BTW: TinyExpr++ seems to need this newline -> \n}{/*Zero point three:*/ 0.3}"
        "/* Second device: */mid360 invalidaddress\n"
        "{ lidar.coord_indexed.x(0) }\n"
        "{1}\n";

    try
    {
        static QMap<EFG::Device, std::shared_ptr<PointFilter::ExpressionFilter_Base> > filters;

        filters = EFG::generateMap(plainText);

        QFAIL("Should throw an exception");
    }
    catch (EFG::Issue issue)
    {
        QCOMPARE(issue.text, "Can't convert parameter \"invalidaddress\" to IPv4 address.");
    }
    catch (...)
    {
        QFAIL("Should not throw any other exception.");
    }

    plainText =
        "/*First device: */\n"
        "mid360 2001:0db8:85a3:0000:0000:8a2e:0370:7334\n" /* IPv6 address */
        "{0.5 // BTW: TinyExpr++ seems to need this newline -> \n}{/*Zero point three:*/ 0.3}"
        "/* Second device: */mid360 invalidaddress\n"
        "{ lidar.coord_indexed.x(0) }\n"
        "{1}\n";

    try
    {
        static QMap<EFG::Device, std::shared_ptr<PointFilter::ExpressionFilter_Base> > filters;

        filters = EFG::generateMap(plainText);

        QFAIL("Should throw an exception");
    }
    catch (EFG::Issue issue)
    {
        QCOMPARE(issue.text, "Parameter \"2001:0db8:85a3:0000:0000:8a2e:0370:7334\" is not a valid IPv4 address.");
    }
    catch (...)
    {
        QFAIL("Should not throw any other exception.");
    }
}

void TestExpressionFilterGenerator::error_ExpressionBlocks()
{
    QString plainText =
        "mid360 1.2.3.4 illegal text{0.5}{0.3}\n"
        "mid360 2.3.4.5 {1}{0.5}";

    try
    {
        static QMap<EFG::Device, std::shared_ptr<PointFilter::ExpressionFilter_Base> > filters;

        filters = EFG::generateMap(plainText);

        QFAIL("Should throw an exception");
    }
    catch (EFG::Issue issue)
    {
        QCOMPARE(issue.text, "Only comments and whitespaces allowed between device definition and opening curly brace for filter expression.");
    }
    catch (...)
    {
        QFAIL("Should not throw any other exception.");
    }

    plainText =
        "mid360 1.2.3.4 {0.5}{0.3}\n"
        "mid360 2.3.4.5 {1}Illegal text{0.5}";

    try
    {
        static QMap<EFG::Device, std::shared_ptr<PointFilter::ExpressionFilter_Base> > filters;

        filters = EFG::generateMap(plainText);

        QFAIL("Should throw an exception");
    }
    catch (EFG::Issue issue)
    {
        QCOMPARE(issue.text, "Only comments and whitespaces allowed between closing and opening curly braces for filter and quality expression.");
    }
    catch (...)
    {
        QFAIL("Should not throw any other exception.");
    }

    plainText =
        "mid360 1.2.3.4 {0.5}{0.3}\n"
        "mid360 2.3.4.5 /* nothing to see here */ ";

    try
    {
        static QMap<EFG::Device, std::shared_ptr<PointFilter::ExpressionFilter_Base> > filters;

        filters = EFG::generateMap(plainText);

        QFAIL("Should throw an exception");
    }
    catch (EFG::Issue issue)
    {
        QCOMPARE(issue.text, "Expression definitions missing for device \"mid360 2.3.4.5\".");
    }
    catch (...)
    {
        QFAIL("Should not throw any other exception.");
    }

    plainText =
        "mid360 1.2.3.4 {0.5}{0.3}\n"
        "mid360 2.3.4.5 {1} /* nothing to see here */ ";

    try
    {
        static QMap<EFG::Device, std::shared_ptr<PointFilter::ExpressionFilter_Base> > filters;

        filters = EFG::generateMap(plainText);

        QFAIL("Should throw an exception");
    }
    catch (EFG::Issue issue)
    {
        QCOMPARE(issue.text, "Quality expression definition missing for device \"mid360 2.3.4.5\".");
    }
    catch (...)
    {
        QFAIL("Should not throw any other exception.");
    }

    plainText =
        "mid360 1.2.3.4 {0.5}{0.3}\n"
        "mid360 2.3.4.5 {1 /* Unterminated block */ ";

    try
    {
        static QMap<EFG::Device, std::shared_ptr<PointFilter::ExpressionFilter_Base> > filters;

        filters = EFG::generateMap(plainText);

        QFAIL("Should throw an exception");
    }
    catch (EFG::Issue issue)
    {
        QCOMPARE(issue.text, "Unterminated block (matching \"}\"-character missing).");
    }
    catch (...)
    {
        QFAIL("Should not throw any other exception.");
    }

    plainText = QString::fromUtf8(
        "mid360 2.3.4.5 {0.5 Hello world in chinese: \xe4\xb8\x96\xe7\x95\x8c\xe6\x82\xa8\xe5\xa5\xbd } {7}"
        "mid360 1.2.3.4 {0.5}{0.3}\n");

    try
    {
        static QMap<EFG::Device, std::shared_ptr<PointFilter::ExpressionFilter_Base> > filters;

        filters = EFG::generateMap(plainText);

        QFAIL("Should throw an exception");
    }
    catch (EFG::Issue issue)
    {
        QCOMPARE(issue.text, "Error compiling filter expression: Only Latin 1 (ISO/IEC 8859-1 / \"8-bit ASCII\") characters allowed in non-comment sections of an expression.");
    }
    catch (...)
    {
        QFAIL("Should not throw any other exception.");
    }

    plainText = QString::fromUtf8(
        "mid360 1.2.3.4 {0.5}{0.3}\n"
        "mid360 2.3.4.5 {1}{0.5 Hello world in chinese: \xe4\xb8\x96\xe7\x95\x8c\xe6\x82\xa8\xe5\xa5\xbd }");

    try
    {
        static QMap<EFG::Device, std::shared_ptr<PointFilter::ExpressionFilter_Base> > filters;

        filters = EFG::generateMap(plainText);

        QFAIL("Should throw an exception");
    }
    catch (EFG::Issue issue)
    {
        QCOMPARE(issue.text, "Error compiling quality expression: Only Latin 1 (ISO/IEC 8859-1 / \"8-bit ASCII\") characters allowed in non-comment sections of an expression.");
    }
    catch (...)
    {
        QFAIL("Should not throw any other exception.");
    }

    plainText =
        "mid360 1.2.3.4 {0.5}{0.3}\n"
        "mid360 2.3.4.5 {parser error}{1}";

    try
    {
        static QMap<EFG::Device, std::shared_ptr<PointFilter::ExpressionFilter_Base> > filters;

        filters = EFG::generateMap(plainText);

        QFAIL("Should throw an exception");
    }
    catch (EFG::Issue issue)
    {
        QCOMPARE(issue.text, "Error compiling filter expression: TinyExpr error: (empty)"); // Does TinyExpr++ ever return any error message?
    }
    catch (...)
    {
        QFAIL("Should not throw any other exception.");
    }

    plainText =
        "mid360 1.2.3.4 {0.5}{0.3}\n"
        "mid360 2.3.4.5 {1}{errrrrrr}";

    try
    {
        static QMap<EFG::Device, std::shared_ptr<PointFilter::ExpressionFilter_Base> > filters;

        filters = EFG::generateMap(plainText);

        QFAIL("Should throw an exception");
    }
    catch (EFG::Issue issue)
    {
        QCOMPARE(issue.text, "Error compiling quality expression: TinyExpr error: (empty)"); // Does TinyExpr++ ever return any error message?
    }
    catch (...)
    {
        QFAIL("Should not throw any other exception.");
    }
}












