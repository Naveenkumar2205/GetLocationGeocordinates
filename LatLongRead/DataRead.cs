using System;
using canlibCLSNET;
using System.Device.Location;
using System.Collections.Generic;
using System.Security.Cryptography;
using ENCX;
using System.Data;

namespace LatLongRead
{
    public class DataRead
    {
        public static GeoCoordinate _USVGeoCoordinate { get; set; } = new GeoCoordinate();
        public static double USV_Angle { get; set; }
        public static double USV_SOG { get; set; }
        public static decimal depth { get; set; }

        //public static double Deg_Rad(double angle)
        //{
        //    return angle / 180.0 * Math.PI;
        //}
        //public static double Rad_Deg(double angle)
        //{
        //    return angle * 180.0 / Math.PI;
        //}
        public static double ToRad(double degrees)
        {
            return degrees * (Math.PI / 180.0);
        }
        public static double ToDegrees(double radians)
        {
            return radians * 180.0 / Math.PI;
        }
        public static double Rad_Deg(double angle)
        {
            return angle * 180.0 / Math.PI;
        }
        public static double ToBearing(double radians)
        {
            // convert radians to degrees (as bearing: 0...360)
            return (ToDegrees(radians) + 360) % 360;
        }
        public static double DegreeBearing(double lat1, double lon1, double lat2, double lon2)
        {
            var dLon = ToRad(lon2 - lon1);
            var dPhi = Math.Log(
                Math.Tan(ToRad(lat2) / 2 + Math.PI / 4) / Math.Tan(ToRad(lat1) / 2 + Math.PI / 4));
            if (Math.Abs(dLon) > Math.PI)
                dLon = dLon > 0 ? -(2 * Math.PI - dLon) : 2 * Math.PI + dLon;
            return ToBearing(Math.Atan2(dLon, dPhi));
        }
        public static double GetDistanceBetweenPoints(double lat1, double long1, double lat2, double long2)
        {
            double num = ToRad(lat2 - lat1);
            double num2 = ToRad(long2 - long1);
            double num3 = Math.Sin(num / 2.0) * Math.Sin(num / 2.0) + Math.Cos(ToRad(lat1)) * Math.Cos(ToRad(lat2)) * Math.Sin(num2 / 2.0) * Math.Sin(num2 / 2.0);
            double num4 = 2.0 * Math.Atan2(Math.Sqrt(num3), Math.Sqrt(1.0 - num3));
            return 6371008.8 * num4;
        }
        public static float GetTotalDistance(List<GeoCoordinate> geoPoints)
        {
            float totalDistance = 0;
            for (int i = 0; i < geoPoints.Count - 1; i++)
            {
                totalDistance = totalDistance + DataRead.DistanceBetweenPoints(geoPoints[i], geoPoints[i + 1]);
            }
            return totalDistance;
        }
        public static GeoCoordinate FindPointAtDistanceFrom(GeoCoordinate startPoint, double initialBearingRadians,
           double distanceKilometers)
        {
            const double radiusEarthKilometers = 6371.01;
            var distRatio = distanceKilometers / radiusEarthKilometers;
            var distRatioSine = Math.Sin(distRatio);
            var distRatioCosine = Math.Cos(distRatio);

            var startLatRad = ToRad(startPoint.Latitude);
            var startLonRad = ToRad(startPoint.Longitude);

            var startLatCos = Math.Cos(startLatRad);
            var startLatSin = Math.Sin(startLatRad);

            var endLatRads = Math.Asin(startLatSin * distRatioCosine +
                                       startLatCos * distRatioSine * Math.Cos(initialBearingRadians));

            var endLonRads = startLonRad
                             + Math.Atan2(
                                 Math.Sin(initialBearingRadians) * distRatioSine * startLatCos,
                                 distRatioCosine - startLatSin * Math.Sin(endLatRads));

            return new GeoCoordinate(ToDegrees(endLatRads), ToDegrees(endLonRads));
        }

        public static string[] GetCardinalCoordinates(GeoPoint gp)
        {
            string[] cardinalString = new string[2];
            string[] latLon = gp.LatLonString.Split(',');

            if (latLon.Length == 2 && double.TryParse(latLon[0], out double latitude) && double.TryParse(latLon[1], out double longitude))
            {
                string latitudeDirection = latitude >= 0 ? "N" : "S";
                string longitudeDirection = longitude >= 0 ? "E" : "W";

                string formattedLatitude = $"{Math.Abs(Math.Round(latitude, 5))}° {latitudeDirection}";
                string formattedLongitude = $"{Math.Abs(Math.Round(longitude, 5))}° {longitudeDirection}";

                cardinalString[0] = formattedLatitude;
                cardinalString[1] = formattedLongitude;
            }
            else
            {
                cardinalString[0] = "Invalid Latitude";
                cardinalString[1] = "Invalid Longitude";
            }
            return cardinalString;
        }

        public static GeoPoint GetGeoPointFromDataRow(DataRow waypointRow)
        {
            char[] directionChars = new char[] { '°', 'N', 'S', 'E', 'W' };

            int latitudeIndex = waypointRow["Latitude"].ToString().IndexOfAny(directionChars);
            double latitude = Convert.ToDouble(waypointRow["Latitude"].ToString().Substring(0, latitudeIndex));

            if (waypointRow["Latitude"].ToString().Contains("S"))
            {
                latitude = -latitude;
            }

            int longitudeIndex = waypointRow["Longitude"].ToString().IndexOfAny(directionChars);
            double longitude = Convert.ToDouble(waypointRow["Longitude"].ToString().Substring(0, longitudeIndex));

            if (waypointRow["Longitude"].ToString().Contains("W"))
            {
                longitude = -longitude;
            }

            return new GeoPoint
            {
                Lat = latitude,
                Lon = longitude
            };
        }

        public static string[] GetCardinalCoordinates(GeoCoordinate gc)
        {
            string[] cardinalString = new string[2];

            string latitudeDirection = gc.Latitude >= 0 ? "N" : "S";
            string longitudeDirection = gc.Longitude >= 0 ? "E" : "W";

            string formattedLatitude = $"{Math.Abs(Math.Round(gc.Latitude, 5))}° {latitudeDirection}";
            string formattedLongitude = $"{Math.Abs(Math.Round(gc.Longitude, 5))}° {longitudeDirection}";

            cardinalString[0] = formattedLatitude;
            cardinalString[1] = formattedLongitude;

            return cardinalString;
        }
        private static decimal Combine(byte b1, byte b2, byte b3, byte b4)
        {
            return (b4 << 24) | (b3 << 16) | (b2 << 8) | b1;
        }
        private static double Mix(byte b1, byte b2)
        {
            return  (b2 << 8) | b1;
        }
        private static int Bin_Hex(string bin)
        {
            int hex = Convert.ToInt32(bin, 2);
            return hex;
        }
        private static uint largeCombine(byte b1, byte b2, byte b3, byte b4)
        {
            uint combinedValue = (uint)((b1) | (b2 << 8) | (b3 << 16) | (b4 << 24));
            return combinedValue;
        }
        // Method to calculate distance between two points
        public static float DistanceBetweenPoints(GeoCoordinate point1, GeoCoordinate point2)
        {
            var sCoord = new GeoCoordinate(point1.Latitude, point1.Longitude);
            var eCoord = new GeoCoordinate(point2.Latitude, point2.Longitude);

            return (float)sCoord.GetDistanceTo(eCoord);
        }
        public static void DumpMessageLoop()
        {
            int handle;
            Canlib.canStatus status;
            int channelNumber = 2;

            Canlib.canInitializeLibrary();

            handle = Canlib.canOpenChannel(channelNumber, Canlib.canOPEN_ACCEPT_VIRTUAL);
            status = Canlib.canSetBusParams(handle, Canlib.canBITRATE_250K, 0, 0, 0, 0, 0);
            status = Canlib.canBusOn(handle);

            byte[] data = new byte[8];
            int id;
            int dlc;
            int flags;
            long timestamp;

            while (true)
            {
                status = Canlib.canReadWait(handle, out id, data, out dlc, out flags, out timestamp, 100);

                string idBinary = Convert.ToString(id, 2).PadLeft(29, '0');

                string hex1 = idBinary.Substring(0, 3);
                string hex2 = idBinary.Substring(3, 18);
                string hex3 = idBinary.Substring(21, 8);

                int h1 = Bin_Hex(hex1);
                int h2 = Bin_Hex(hex2);
                int h3 = Bin_Hex(hex3);
                if (status == Canlib.canStatus.canOK)
                {
                    if (h2 == 129025)
                    {
                        _USVGeoCoordinate.Latitude = Convert.ToDouble(Combine(data[0], data[1], data[2], data[3]) / 10000000m);
                        _USVGeoCoordinate.Longitude = Convert.ToDouble(Combine(data[4], data[5], data[6], data[7]) / 10000000m);
                    }
                    if(h2 == 127250)
                    {
                        USV_Angle = Mix(data[1], data[2]);
                        USV_Angle = Math.Round(ToDegrees(USV_Angle / 10000.0), 1);
                    }
                    if(h2 == 129026)
                    {
                        USV_SOG = Mix(data[4], data[5]) * 2/100.0;
                    }
                    if (h2 == 128267)
                    {
                        depth = Combine(data[0], data[1], data[2], data[3]) / 26000;
                    }
                }
            }
        }
    }
}