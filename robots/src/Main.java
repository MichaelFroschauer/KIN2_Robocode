import java.io.File;
import java.io.IOException;
import java.nio.file.*;
import java.security.IdentityScope;
import java.util.List;

public class Main {
    public static void main(String[] args) throws IOException {
        var source = Path.of("out/production/robots");
        var destinations = List.of("../robocode/robots", "../robocode-windows/robots");
        var root = Path.of("");
        var rootAbs = root.toAbsolutePath();

        List<Path> files;
        for (var targetDir : destinations) {
            var targetRoot = rootAbs.resolve(targetDir).normalize();
            try (var stream = Files.walk(source, FileVisitOption.FOLLOW_LINKS)) {
                stream.filter(x -> !x.getFileName().toString().equals("Main.class")).forEach(path -> {
                    var p = source.relativize(path);
                    var file = path.toFile();
                    var targetPath = targetRoot.resolve(p).normalize().toAbsolutePath();
                    var targetFile = targetPath.toFile();

                    if(file.isDirectory()) {
                        System.out.println("Creating directory " + targetFile);
                        targetFile.mkdir();
                    } else {
                        System.out.println("Copying file " + file + " to " + targetFile);
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
}
