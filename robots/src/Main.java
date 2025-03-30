import java.io.File;
import java.io.IOException;
import java.nio.file.*;

public class Main {
    public static void main(String[] args) throws IOException {
        String source = "out/production/robots";
        String dest = "robocode/robots";

        Path sourcePath = new File(source).toPath();

        try (var stream = Files.walk(sourcePath, FileVisitOption.FOLLOW_LINKS)) {
            stream.forEach(path -> {
                File file = path.toFile();
                String target = file.toString().replace(source, dest);
                File targetFile = new File(target);
                if (file.isDirectory()) {
                    System.out.println("Creating directory " + targetFile);
                    targetFile.mkdir();
                } else {
                    System.out.println("Copying file " + targetFile);
                    try {
                        Files.copy(file.toPath(), targetFile.toPath(), StandardCopyOption.REPLACE_EXISTING);
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                }
            });
        }

    }
}
