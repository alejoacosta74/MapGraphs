package basicgraph;


import javax.net.ssl.HttpsURLConnection;
import java.io.*;
import java.net.MalformedURLException;
import java.net.URL;
import java.net.URLConnection;

public class speedMap {

    static private URL myURL;



    static public void main(String args[]) throws IOException {
        try {


            URL myUrl = new URL(roadsGoogleUrl);
            HttpsURLConnection conn = (HttpsURLConnection) myUrl.openConnection();
            InputStream is = conn.getInputStream();
            InputStreamReader isr = new InputStreamReader(is);
            BufferedReader br = new BufferedReader(isr);

            String inputLine;

            while ((inputLine = br.readLine()) != null) {
                System.out.println(inputLine);
            }

            br.close();
        } catch (Exception e) {
            e.printStackTrace();
        }

    }
}
